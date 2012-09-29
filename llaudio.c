/* Low latency audio interception for Android 
 *
 *  This file implements a low level interceptor for audio, as we are striving to get the lowest
 *   possible latency on android devices. The price to pay is that we can't control sampling rate
 *   or channels captured, or the buffer size. But, we will get the lowest latency in return.
 *
 * (C) 2012 Eduardo José Tagle <ejtagle@tutopia.com>
 *
 */
 
struct audio_module* am;

struct audio_hw_device* ahd;
am->common.methods->open()

ahd->get_input_buffer_size();
ahd->open_input_stream();
ahd->close_input_stream();

struct audio_stream_in