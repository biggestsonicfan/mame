// license:BSD-3-Clause
// copyright-holders:Grant Galitz, Katelyn Gadd
/***************************************************************************

	JSMAME web audio backend v0.4

	Original by katelyn gadd - kg at luminance dot org ; @antumbral on twitter
	Substantial changes by taisel

***************************************************************************/

var jsmame_web_audio = (function () {

var context = null;
var gain_node = null;
// MAME renders at its configured sample rate (-samplerate, 48000 by default) and never asks the
// browser for its own, so the context is opened at that rate and the browser resamples. With
// a 44.1 kHz device and a 48 kHz context MAME fed 9% more than was played: the ring below sat
// full (~0.45 s behind) and kept dropping samples.
var MAME_RATE = 48000;
var BLOCK = 2048;                  // frames per audio callback (~43 ms)
var MAX_FRAMES = BLOCK * 4;        // latency cap: drop the oldest audio beyond this
var PRIME_FRAMES = BLOCK * 3 / 2;  // after an underrun, wait for this much before playing
var playing = false;
var underruns = 0;
var lastL = 0, lastR = 0;
var eventNode = null;
var sampleScale = 32766;
var inputBuffer = new Float32Array(44100);
var bufferSize = 44100;
var start = 0;
var rear = 0;
var watchDogDateLast = null;
var watchDogTimerEvent = null;

function lazy_init () {
	//Make
	if (context) {
		//Return if already created:
		return;
	}
	if (typeof AudioContext != "undefined") {
		//Standard context creation, at MAME's rate:
		try {
			context = new AudioContext({ sampleRate: MAME_RATE, latencyHint: "interactive" });
		} catch (e) {
			context = new AudioContext();
		}
	}
	else if (typeof webkitAudioContext != "undefined") {
		//Older webkit context creation:
		context = new webkitAudioContext();
	}
	else {
		//API not found!
		return;
	}
	//Generate a volume control node:
	gain_node = context.createGain();
	//Set initial volume to 1:
	gain_node.gain.value = 1.0;
	//Connect volume node to output:
	gain_node.connect(context.destination);
	//Initialize the streaming event:
	init_event();
};

function init_event() {
	//Generate a streaming node point:
	if (typeof context.createScriptProcessor == "function") {
		//Current standard compliant way:
		eventNode = context.createScriptProcessor(BLOCK, 0, 2);
	}
	else {
		//Deprecated way:
		eventNode = context.createJavaScriptNode(BLOCK, 0, 2);
	}
	//Make our tick function the audio callback function:
	eventNode.onaudioprocess = tick;
	//Connect stream to volume control node:
	eventNode.connect(gain_node);
	//Workarounds for browser issues:
	initializeWatchDog();
};

function initializeWatchDog() {
	watchDogDateLast = (new Date()).getTime();
	if (watchDogTimerEvent === null) {
		watchDogTimerEvent = setInterval(function () {
			var timeDiff = (new Date()).getTime() - watchDogDateLast;
			if (timeDiff > 500) {
				//WORKAROUND FOR FIREFOX BUG:
				//TODO: decide if we want to user agent sniff Firefox here,
				//since Google Chrome doesn't need this:
				disconnect_old_event();
				init_event();

				//Work around autoplay restrictions in Chrome 71+ https://developers.google.com/web/updates/2017/09/autoplay-policy-changes#webaudio
				if (context) {
					context.resume();
				}
			}
		}, 500);
	}
};

function disconnect_old_event() {
	//Disconnect from audio graph:
	eventNode.disconnect();
	//IIRC there was a firefox bug that did not GC this event when nulling the node itself:
	eventNode.onaudioprocess = null;
	//Null the glitched/unused node:
	eventNode = null;
};

function stream_sink_update (
	pBuffer,           // pointer into emscripten heap. int16 samples
	samples_this_frame // int. number of samples at pBuffer address.
) {
	lazy_init();
	if (!context) return;

	for (
		var i = 0,
		l = samples_this_frame | 0;
		i < l;
		i++
	) {
		var offset =
			// divide by sizeof(int16_t) since pBuffer is offset
			//  in bytes
			((pBuffer / 2) | 0) +
			((i * 2) | 0);

		var left_sample = HEAP16[offset];
		var right_sample = HEAP16[(offset + 1) | 0];

		// normalize from signed int16 to signed float
		var left_sample_float = left_sample / sampleScale;
		var right_sample_float = right_sample / sampleScale;

		inputBuffer[rear++] = left_sample_float;
		inputBuffer[rear++] = right_sample_float;
		if (rear == bufferSize) {
			rear = 0;
		}
		if (start == rear) {
			start += 2;
			if (start == bufferSize) {
				start = 0;
			}
		}
	}
	//Keep the delay bounded: beyond MAX_FRAMES, drop the oldest audio (one jump, not a
	//steady trickle of drops as the ring overflows):
	var count = rear - start;
	if (count < 0) count += bufferSize;
	if (count > MAX_FRAMES * 2) {
		start = rear - PRIME_FRAMES * 2;
		if (start < 0) start += bufferSize;
	}
};

function tick (event) {
	//Find all output channels:
	for (var bufferCount = 0, buffers = []; bufferCount < 2; ++bufferCount) {
		buffers[bufferCount] = event.outputBuffer.getChannelData(bufferCount);
	}
	//Copy samples from the input buffer to the Web Audio API:
	//Jitter buffer: MAME hands over audio a frame at a time, unevenly; after running dry,
	//stay silent until a cushion of PRIME_FRAMES has built up again:
	var have = rear - start;
	if (have < 0) have += bufferSize;
	if (!playing && have >= PRIME_FRAMES * 2) playing = true;
	var index = 0;
	if (playing) for (; index < BLOCK && start != rear; ++index) {
		lastL = buffers[0][index] = inputBuffer[start++];
		lastR = buffers[1][index] = inputBuffer[start++];
		if (start == bufferSize) {
			start = 0;
		}
	}
	//Underrun: fade from the last sample to silence (holding it was a click and a DC step):
	if (index < BLOCK) {
		if (playing) underruns++;
		playing = false;
		for (var k = 0; index < BLOCK; ++index, ++k) {
			var g = k < 256 ? 1 - k / 256 : 0;
			buffers[0][index] = lastL * g;
			buffers[1][index] = lastR * g;
		}
		lastL = lastR = 0;
	}
	//Deep inside the bowels of vendors bugs,
	//we're using watchdog for a firefox bug,
	//where the user agent decides to stop firing events
	//if the user agent lags out due to system load.
	//Don't even ask....
	watchDogDateLast = (new Date()).getTime();
}

function get_context() {
	return context;
};

function sample_count() {
	//TODO get someone to call this from the emulator,
	//so the emulator can do proper audio buffering by
	//knowing how many samples are left:
	if (!context) {
		//Use impossible value as an error code:
		return -1;
	}
	var count = rear - start;
	if (start > rear) {
		count += bufferSize;
	}
	return count;
}

function underrun_count() {
	return underruns;
}

return {
	stream_sink_update: stream_sink_update,
	get_context: get_context,
	sample_count: sample_count,
	underrun_count: underrun_count
};

})();

window.jsmame_stream_sink_update = jsmame_web_audio.stream_sink_update;
window.jsmame_sample_count = jsmame_web_audio.sample_count;
window.jsmame_underrun_count = jsmame_web_audio.underrun_count;
