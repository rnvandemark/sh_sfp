#!/usr/bin/env python3

from __future__ import unicode_literals
from sys import argv as sargv
from enum import Enum
from collections import deque
from threading import Lock
from os.path import exists as oexists
from essentia.standard import MonoLoader, EqloudLoader, OnsetDetection, \
    Windowing, FFT, CartesianToPolar, FrameGenerator, Onsets, \
    RhythmExtractor2013, PredominantPitchMelodia
from essentia import Pool, array as earr

from rclpy import init as rclpy_init, spin as rclpy_spin
from rclpy.executors import MultiThreadedExecutor   
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

import sh_common_constants
from sh_common.heartbeat_node import HeartbeatNode
from sh_sfp_interfaces.action import AnalyzeSoundFile
from sh_sfp_interfaces.msg import OnsetDetectionAlgorithms, \
    WindowingAlgorithms, RhythmDetectionAlgorithms

# Assume constant?
SAMPLE_RATE = 44100

## Helper function to get a string of a goal handle's UUID as a string
#  @param goal_handle The goal handle.
#  @return The UUID of the handle, stringified from the list of bytes.
def uuid2str(goal_handle):
    return str(list(goal_handle.goal_id.uuid.tobytes()))

## Perform onset detection on the input audio to find the timestamps of onsets
#  (significant differences in the audio, outside of just a beat).
#  @param audio The loaded audio to analyze.
#  @param onset_type One of OnsetDetectionAlgorithmTypes.
#  @param window_type One of WindowingAlgorithmTypes.
#  @param frame_size The frame size of the frame generator.
#  @param hop_size The hop size of the frame generator.
#  @return A numpy array of timestamps where onsets occur.
def do_onset_detection(audio, onset_type, window_type, frame_size=1024, hop_size=512):
    onset_detection = OnsetDetection(method=str(onset_type))
    windowing = Windowing(type=str(window_type))
    fft = FFT()
    c2p = CartesianToPolar()
    pool = Pool()
    feature = "features.{0}".format(str(onset_type))
    for frame in FrameGenerator(audio, frameSize=frame_size, hopSize=hop_size):
        mag, phase = c2p(fft(windowing(frame)))
        pool.add(feature, onset_detection(mag, phase))
    return Onsets()(earr([pool[feature]]), [1])

## Perform beat detection on the input audio to find the timstamps of when a
#  beat/rhythm occurs.
#  @param audio The loaded audio to analyze.
#  @param beat_type One of RhythmDetectionAlgorithmTypes.
#  @return A numpy array of timestamps and the confidence of those timestamps.
def do_beat_detection(audio, beat_type):
    _, beats, beats_conf, _, _ = RhythmExtractor2013(method=str(beat_type))(audio)
    return beats, beats_conf

## Perform pitch detection on the input audio to find the pitch over the
#  duration of the audio.
#  @param audio The loaded audio to analyze.
#  @return A numpy array of pitches and the confidence of those timestamps.
def do_pitch_detection(audio):
    return PredominantPitchMelodia(frameSize=1024, hopSize=512)(audio)

## Get the pitches corresponding to a series of timestamps.
#  @param pitches The array of pitches over the duration of the entire audio.
#  @param marks The list of timestamps to get the pitches of.
#  @param audio_seconds The number of seconds the audio lasts.
#  @return A list of pitches corresponding to each mark/timestamp.
def get_pitches_at(pitches, marks, audio_seconds):
    return [pitches[int((m/audio_seconds) * len(pitches))].item() for m in marks]

## The possible onset detection algorithms.
class OnsetDetectionAlgorithmTypes(Enum):
    HFC = OnsetDetectionAlgorithms.HFC
    COMPLEX = OnsetDetectionAlgorithms.COMPLEX
    COMPLEX_PHASE = OnsetDetectionAlgorithms.COMPLEX_PHASE
    FLUX = OnsetDetectionAlgorithms.FLUX
    MELFLUX = OnsetDetectionAlgorithms.MELFLUX
    RMS = OnsetDetectionAlgorithms.RMS

    ## str() operator overload.
    #  @param self The object pointer.
    #  @return The string name of the algorithm.
    def __str__(self):
        if self == OnsetDetectionAlgorithmTypes.HFC:
            return "hfc"
        elif self == OnsetDetectionAlgorithmTypes.COMPLEX:
            return "complex"
        elif self == OnsetDetectionAlgorithmTypes.COMPLEX_PHASE:
            return "complex_phase"
        elif self == OnsetDetectionAlgorithmTypes.FLUX:
            return "flux"
        elif self == OnsetDetectionAlgorithmTypes.MELFLUX:
            return "melflux"
        elif self == OnsetDetectionAlgorithmTypes.RMS:
            return "rms"
        else:
            return ""

## The possible windowing detection algorithms.
class WindowingAlgorithmTypes(Enum):
    HAMMING = WindowingAlgorithms.HAMMING
    HANN = WindowingAlgorithms.HANN
    HANNNSGCQ = WindowingAlgorithms.HANNNSGCQ
    TRIANGULAR = WindowingAlgorithms.TRIANGULAR
    SQUARE = WindowingAlgorithms.SQUARE
    BLACKMANHARRIS62 = WindowingAlgorithms.BLACKMANHARRIS62
    BLACKMANHARRIS70 = WindowingAlgorithms.BLACKMANHARRIS70
    BLACKMANHARRIS74 = WindowingAlgorithms.BLACKMANHARRIS74

    ## str() operator overload.
    #  @param self The object pointer.
    #  @return The string name of the algorithm.
    def __str__(self):
        if self == WindowingAlgorithmTypes.HAMMING:
            return "hamming"
        elif self == WindowingAlgorithmTypes.HANN:
            return "hann"
        elif self == WindowingAlgorithmTypes.HANNNSGCQ:
            return "hannnsgcq"
        elif self == WindowingAlgorithmTypes.TRIANGULAR:
            return "triangular"
        elif self == WindowingAlgorithmTypes.SQUARE:
            return "square"
        elif self == WindowingAlgorithmTypes.BLACKMANHARRIS62:
            return "blackmanharris62"
        elif self == WindowingAlgorithmTypes.BLACKMANHARRIS70:
            return "blackmanharris70"
        elif self == WindowingAlgorithmTypes.BLACKMANHARRIS74:
            return "blackmanharris74"
        else:
            return ""

## The possible rhythm detection algorithms.
class RhythmDetectionAlgorithmTypes(Enum):
    MULTIFEATURE = RhythmDetectionAlgorithms.MULTIFEATURE
    DEGARA = RhythmDetectionAlgorithms.DEGARA

    ## str() operator overload.
    #  @param self The object pointer.
    #  @return The string name of the algorithm.
    def __str__(self):
        if self == RhythmDetectionAlgorithmTypes.MULTIFEATURE:
            return "multifeature"
        elif self == RhythmDetectionAlgorithmTypes.DEGARA:
            return "degara"
        else:
            return ""

## A node dedicated to resolving audio analaysis requests.
class AudioAnalyzerNode(HeartbeatNode):

    ## Constructor.
    #  @param self The object pointer.
    def __init__(self):
        super(AudioAnalyzerNode, self).__init__("sh_audio_analyzer")

        # Declare action
        self.analyze_audio_srv = ActionServer(
            self,
            AnalyzeSoundFile,
            sh_common_constants.actions.ANALYZE_SOUND_FILE,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Declare queue of pending goals, a mutex to access it safely, and a current goal
        self.goal_queue = deque()
        self.goal_queue_mutex = Lock()
        self.current_goal = None

        # Done
        self.get_logger().info("Started.")

    ## Helper function to start a goal's execution.
    #  @param self The object pointer.
    #  @param next_goal_handle The new goal to start execution for.
    def start_goal_execution(self, next_goal_handle):
        self.current_goal = next_goal_handle
        self.get_logger().info("Executing next goal: " + uuid2str(self.current_goal))
        self.current_goal.execute()

    ## Callback to either accept or reject requested analysis. The goal is
    #  rejected if the local file URL does not exist or the implementations of
    #  any algorithm is not a valid value, otherwise it's accepted.
    #  @param self The object pointer.
    #  @param goal_request The requested goal to either accept or reject.
    #  @return The acceptance/rejection status.
    def goal_callback(self, goal_request):
        u = goal_request.local_url
        o = goal_request.onset.alg
        r = goal_request.rhythm.alg
        w = goal_request.window.alg
        resp = GoalResponse.ACCEPT
        if oexists(u):
            try:
                self.get_logger().info(
                    "Received analysis request: '{0}' (onset='{1}', windowing='{2}', rhythm='{3}')".format(
                        u,
                        OnsetDetectionAlgorithmTypes(o),
                        WindowingAlgorithmTypes(w),
                        RhythmDetectionAlgorithmTypes(r)
                    )
                )
            except Exception as e:
                self.get_logger().error(
                    "Invalid analysis request: '{0}' (onset='{1}', windowing='{2}', rhythm='{3}')".format(
                        u,
                        str(o),
                        str(w),
                        str(r)
                    )
                )
                resp = GoalResponse.REJECT
        else:
            self.get_logger().error("Requested path does not exist: '{0}'".format(u))
            resp = GoalResponse.REJECT
        return resp

    ## Callback to handle goals that were accepted. If no goal is being
    #  processed, processing for this one will be. Otherwise, this one is
    #  queued.
    #  @param self The object pointer.
    #  @param goal_handle The handle for the action's goal.
    def handle_accepted_callback(self, goal_handle):
        with self.goal_queue_mutex:
            if self.current_goal is None:
                self.start_goal_execution(goal_handle)
            else:
                self.goal_queue.append(goal_handle)
                self.get_logger().info("New goal queued: " + uuid2str(goal_handle))

    ## Simple callback for requests to cancel the current download. All goals
    #  can be cancelled.
    #  @param self The object pointer.
    #  @param goal_handle The handle for the action's goal.
    #  @return The acceptance status.
    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancelling goal: " + uuid2str(goal_handle))
        return CancelResponse.ACCEPT

    ## Start executing an analysis goal, the next one in the queue.
    #  @param self The object pointer.
    #  @param goal_handle The handle for the action's goal.
    #  @return The goal's result.
    async def execute_callback(self, goal_handle):
        try:
            # Prepare the feedback and a helper function to publish feedback
            feedback_msg = AnalyzeSoundFile.Feedback()
            def pub_feedback(status):
                feedback_msg.status = status
                goal_handle.publish_feedback(feedback_msg)

            # Capture the arguments
            local_url = goal_handle.request.local_url
            onset_alg = OnsetDetectionAlgorithmTypes(goal_handle.request.onset.alg)
            windowing_alg = WindowingAlgorithmTypes(goal_handle.request.window.alg)
            rhythm_alg = RhythmDetectionAlgorithmTypes(goal_handle.request.rhythm.alg)

            # Publish the initial feedback message
            pub_feedback(AnalyzeSoundFile.Feedback.STATUS_STARTED)
            self.get_logger().info("Analyzing audio at '{0}'.".format(local_url))

            # Load the audio in two separate representations and publish an update
            mono_loaded = MonoLoader(filename=local_url, sampleRate=SAMPLE_RATE)()
            eqloud_loaded = EqloudLoader(filename=local_url, sampleRate=SAMPLE_RATE)()
            audio_seconds = len(mono_loaded) / float(SAMPLE_RATE)
            pub_feedback(AnalyzeSoundFile.Feedback.STATUS_AUDIO_LOADED)

            # Do each type of analysis and publish feedback intermittently
            onsets = do_onset_detection(mono_loaded, onset_alg, windowing_alg)
            pub_feedback(AnalyzeSoundFile.Feedback.STATUS_FINISHED_ONSET_DETECTION)
            beats, _ = do_beat_detection(mono_loaded, rhythm_alg)
            pub_feedback(AnalyzeSoundFile.Feedback.STATUS_FINISHED_BEAT_DETECTION)
            pitches, _ = do_pitch_detection(eqloud_loaded)
            pub_feedback(AnalyzeSoundFile.Feedback.STATUS_FINISHED_PITCH_DETECTION)
            onset_pitches = get_pitches_at(pitches, onsets, audio_seconds)
            beat_pitches = get_pitches_at(pitches, beats, audio_seconds)
            pub_feedback(AnalyzeSoundFile.Feedback.STATUS_FINISHED_ANALYSIS)

            # Package the characteristics into the results
            result_msg = AnalyzeSoundFile.Result()
            result_msg.characteristics.onsets.data = onsets.tolist()
            result_msg.characteristics.onset_pitches.data = onset_pitches
            result_msg.characteristics.beats.data = beats.tolist()
            result_msg.characteristics.beat_pitches.data = beat_pitches

            self.get_logger().info("Finished analysis from '{0}'.".format(local_url))
            goal_handle.succeed()

            return result_msg

        finally:
            with self.goal_queue_mutex:
                try:
                    # Try to start executing the next goal in the queue
                    self.start_goal_execution(self.goal_queue.popleft())
                except IndexError:
                    # Exception due to there being no next goal in the queue
                    self.current_goal = None

## The main function to startup the ROS environment and create our node.
def main():
    # Create our node with a multi-threaded executor to process goal requests concurrently
    rclpy_init(args=sargv)
    node = AudioAnalyzerNode()
    try:
        rclpy_spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
