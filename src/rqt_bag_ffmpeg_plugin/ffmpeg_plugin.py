from rqt_bag.plugins.plugin import Plugin

from .ffmpeg_view import FfmpegView

class FfmpegPlugin(Plugin):

    def __init__(self):
        pass

    def get_view_class(self):
        return FfmpegView

    def get_renderer_class(self):
        return None

    def get_message_types(self):
        return ['ffmpeg_image_transport_msgs/msg/FFMPEGPacket']
