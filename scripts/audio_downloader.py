#!/usr/bin/env python3

from __future__ import unicode_literals
from sys import argv as sargv
from enum import Enum
from os import sep as osep
from os.path import join as ojoin
from datetime import datetime
from time import sleep
from collections import deque
from threading import Lock
import youtube_dl
import ffmpeg

from rclpy import init as rclpy_init, spin as rclpy_spin, shutdown as rclpy_shutdown
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor   
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Header

import sh_common_constants
from sh_common.heartbeat_node import HeartbeatNode
from sh_sfp_interfaces.action import DownloadAudio

# Constant: the directory that contains all the downloaded audio
SH_TMP_DIR = ojoin(osep, "tmp", "sh")

## Helper function to get a string of a goal handle's UUID as a string
#  @param goal_handle The goal handle.
#  @return The UUID of the handle, stringified from the list of bytes.
def uuid2str(goal_handle):
    return str(list(goal_handle.goal_id.uuid.tobytes()))

## Helper function to get the file path for a downloaded audio file.
#  @param unique_id The unique ID of the audio.
#  @param file_format The file format / extension.
#  @return The absolute file path.
def get_output_file_path(unique_id, file_format):
    return ojoin(
        SH_TMP_DIR,
        "{0}.{1}".format(unique_id, file_format)
    )

## An enum to help guarantee a proper video quality was specified
class QualityTypes(Enum):
    ## 64 kbps
    _64 = DownloadAudio.Goal.QUALITY_64
    ## 128 kbps
    _128 = DownloadAudio.Goal.QUALITY_128
    ## 192 kbps
    _192 = DownloadAudio.Goal.QUALITY_192
    ## 256 kbps
    _256 = DownloadAudio.Goal.QUALITY_256
    ## 320 kbps
    _320 = DownloadAudio.Goal.QUALITY_320

    ## Convert the enum to the string '64', '128', etc.
    #  @param self The object pointer.
    #  @param return The quality integer as a string.
    def __str__(self):
        return self.name[1:]

## A helper class that contains the status of a download currently occurring asynchronously.
class AsyncDownload():

    ## The constructor. Prepare to start a newly requested download.
    #  @param self The object pointer.
    #  @param video_id The unique ID of the YouTube video to download.
    #  @param quality A QualityTypes value to specify the download quality of the video.
    #  @param local_url The absolute URL on this machine of the file that the downloaded
    #  video is being saved to.
    def __init__(self, video_id, quality, local_url):
        self.finished = False
        self.completion = 0.0
        self.video_id = video_id
        self.quality = quality
        self.local_url = local_url
        self.err_msg = None
        self.success = False  # Prove this otherwise

## A node that hosts an action which can be used to download YouTube videos as .FILE_FORMAT files.
class AudioDownloaderNode(HeartbeatNode):

    ## The constructor. Implements the node heartbeat and creates the action server.
    #  @param self The object pointer.
    def __init__(self):
        super(AudioDownloaderNode, self).__init__("sh_audio_downloader")

        # Declare action
        self.download_audio_srv = ActionServer(
            self,
            DownloadAudio,
            sh_common_constants.actions.DOWNLOAD_AUDIO,
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

    ## Callback to either accept or reject requested downloads. The goal is rejected
    #  only if the given quality is not a valid value, otherwise it's accepted.
    #  @param self The object pointer.
    #  @param goal_request The requested goal to either accept or reject.
    #  @return The acceptance/rejection status.
    def goal_callback(self, goal_request):
        v = goal_request.video_id
        q = goal_request.quality
        resp = GoalResponse.ACCEPT
        try:
            self.get_logger().info(
                "Received download request: '{0}' at {1} kbps".format(v, str(QualityTypes(q)))
            )
        except Exception as e:
            self.get_logger().error("Invalid download request: {0}, {1}".format(v, str(q)))
            resp = GoalResponse.REJECT
        return resp

    ## Callback to either accept or reject requested downloads. The goal is rejected
    #  only if the given quality is not a valid value, otherwise it's accepted.
    #  @param self The object pointer.
    #  @param goal_request The requested goal to either accept or reject.
    #  @return The acceptance/rejection status.
    def handle_accepted_callback(self, goal_handle):
        with self.goal_queue_mutex:
            if self.current_goal is None:
                self.start_goal_execution(goal_handle)
            else:
                self.goal_queue.append(goal_handle)
                self.get_logger().info("New goal queued: " + uuid2str(goal_handle))

    ## Simple callback for requests to cancel the current download. Unfortunately there
    #  is no way with the youtube-dl API to cancel a download, so all requests are rejected.
    #  @todo Allow for queued goals that haven't yet been started to be cancelled.
    #  @param self The object pointer.
    #  @param goal_handle The handle for the action's goal.
    #  @return The rejection status.
    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Cannot cancel a youtube_dl download, ignoring request.")
        return CancelResponse.REJECT

    ## Async callback for updates on the current download. We are guaranteed at least one
    #  callback issued here when the download is finished.
    #  @param self The object pointer.
    #  @param update Telemetry on the download's update.
    async def progress_update_hook(self, update, async_download):
        status = update["status"]
        if "error" == status:
            # There was an error during the download. We're finished, but we log the error.
            async_download.err_msg = "Download failed at {0}%.".format(async_download.completion)
            async_download.finished = True
        elif "downloading" == status:
            # We're still downloading, report the percent downloaded out of 100.
            async_download.completion = float(update["_percent_str"].replace("%", ""))
        elif "finished" == status:
            # We finished the download successfully.
            async_download.completion = 100.0
            async_download.success = True
            async_download.finished = True
        else:
            self.get_logger().warn("Unknown status update type: " + str(status))

        self.get_logger().debug("Received download update: '{0}', {1}%".format(status, async_download.completion))

    ## Asynchronously start the download currently queued.
    #  @param self The object pointer.
    #  @param async_download An instance of AsyncDownload to manage the progress of one download.
    #  @param file_format The desired file format as a string.
    async def do_download(self, async_download, file_format):
        # Safely handle any and all exceptions, simply setting an error message on failure
        try:
            lurl = async_download.local_url
            yt_opts = {
                "quiet": True, # Suppress output to STDOUT
                "format": file_format,
                "extractaudio": True,
                "prefer-ffmpeg": True,
                "postprocessors": [{
                    "key": "FFmpegExtractAudio",
                    "preferredquality": str(async_download.quality), # Use the specified quality
                }],
                "outtmpl": lurl, # Download to the specified local file URL
                "noplaylist": True, # Ignore YouTube playlists if given, only download the current video
                "progress_hooks": [ # Create an async task out of every progress update callback
                    lambda u: self.executor.create_task(self.progress_update_hook(u, async_download))
                ],
            }
            with youtube_dl.YoutubeDL(yt_opts) as ydl:
                # Finally, perform the actual download
                self.get_logger().info("Starting download to '{0}'.".format(lurl))
                ydl.download(["https://www.youtube.com/watch?v=" + async_download.video_id])
                self.get_logger().info("Finished download to '{0}'.".format(lurl))
        except Exception as e:
            self.err_msg = "Exception encountered: " + str(e)
            async_download.finished = True

    ## Async callback for a requested goal having been accepted, so start meeting the
    #  goal. For this action, that means starting the download, publishing intermediate
    #  updates while still downloading, and returning the local file URL where the data
    #  is downloaded to once done.
    #  @param self The object pointer.
    #  @param goal_handle The handle for the action's goal.
    #  @return The result message, DownloadAudio.Result, of the requested download.
    async def execute_callback(self, goal_handle):
        try:
            # Prepare the result
            result_msg = DownloadAudio.Result()

            # Capture the arguments
            # Download with the first file format in the list, then get the rest via conversion
            video_id = goal_handle.request.video_id
            quality = QualityTypes(goal_handle.request.quality)
            file_formats = goal_handle.request.file_formats.data

            # Require at least one desired file format
            if len(file_formats) > 0:
                # Init the resulting paths as all failures, overwrite as necessary
                result_msg.local_urls.data = [""] * len(file_formats)

                # Target the local file URL to be in the smart home temporary folder
                unique_id = "{0}_{1}".format(
                    datetime.now().strftime("%Y%m%dT%H%M%S%f"),
                    video_id
                )
                first_file_format = file_formats[0]
                first_local_url = get_output_file_path(unique_id, first_file_format)

                # Set the download configuration parameters and start the async download
                async_download = AsyncDownload(video_id, quality, first_local_url)
                self.executor.create_task(self.do_download(async_download, first_file_format))

                # Start publishing feedback until the download is complete
                feedback_msg = DownloadAudio.Feedback()
                prev_completion = 0.0
                in_progress = True
                while in_progress:
                    sleep(0.5)
                    curr_completion = async_download.completion
                    # Only output an update if the percent complete changed
                    if prev_completion != curr_completion:
                        feedback_msg.completion = curr_completion
                        goal_handle.publish_feedback(feedback_msg)
                        prev_completion = curr_completion
                    # Loop until the download has been marked as finished
                    in_progress = not async_download.finished

                # If the download was successful, attempt conversion to each file format
                # If a conversion fails, denote this in the result as an empty string (per
                # the specification of the action's definition)
                if async_download.success:
                    result_msg.local_urls.data[0] = first_local_url
                    for ii, ff in enumerate(file_formats[1:], start=1):
                        next_local_url = get_output_file_path(unique_id, ff)
                        try:
                            # Attempt to convert to the specified file format
                            # There is already an empty string at this index in the result,
                            # so only bother overwriting on success
                            ffmpeg.input(first_local_url).output(next_local_url).run(
                                capture_stdout=True,
                                capture_stderr=True,
                                overwrite_output=True
                            )
                            result_msg.local_urls.data[ii] = next_local_url
                            self.get_logger().info(
                                "Successfully converted '{0}' to '{1}' format.".format(first_local_url, ff)
                            )
                        except ffmpeg._run.Error:
                            # If the conversion failed, ignore it
                            self.get_logger().info(
                                "Failed to convert '{0}' to '{1}' format.".format(first_local_url, ff)
                            )
                            pass

                    # Finally, mark the action as a success
                    goal_handle.succeed()
                else:
                    # The download failed, log the error message
                    self.get_logger().error(async_download.err_msg)
                    goal_handle.abort()
            else:
                self.get_logger().error("Zero desired file formats were specified.")
                goal_handle.abort()

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
    node = AudioDownloaderNode()
    try:
        rclpy_spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
