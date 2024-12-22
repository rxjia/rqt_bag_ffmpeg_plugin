# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import threading
from PIL import Image
# HACK workaround for upstream pillow issue python-pillow/Pillow#400
import sys
from python_qt_binding import QT_BINDING_MODULES
if (
    not QT_BINDING_MODULES['QtCore'].__name__.startswith('PyQt5') and
    'PyQt5' in sys.modules
):
    sys.modules['PyQt5'] = None
from PIL.ImageQt import ImageQt
from rqt_bag import TopicMessageView
from rqt_bag import bag_helper
from .bag_video_decoder import BagVideoDecoder

from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtCore import qDebug, Signal
from python_qt_binding.QtWidgets import QGraphicsScene, QGraphicsView

# rclpy used for Time and Duration objects, for interacting with rosbag
from rclpy.duration import Duration
from rclpy.time import Time


class FfmpegView(TopicMessageView):

    """
    Popup image viewer
    """
    name = 'Image'
    resize_signal = Signal()

    def __init__(self, timeline, parent, topic):
        super(FfmpegView, self).__init__(timeline, parent, topic)
        qDebug("FfmpegView.__init__...")
        
        self.lock = threading.Lock()

        self.decode_video(topic)

        self._image = None
        self._image_topic = None
        self._image_stamp = None
        self.quality = Image.NEAREST  # quality hint for scaling

        # TODO put the image_topic and image_stamp on the picture or display them in some fashion
        self._overlay_font_size = 14.0
        self._overlay_indent = (4, 4)
        self._overlay_color = (0.2, 0.2, 1.0)

        self.resize_signal.connect(self._resize_cb)
        self._image_view = QGraphicsView(parent)
        self._image_view.resizeEvent = self._resizeEvent
        self._scene = QGraphicsScene()
        self._image_view.setScene(self._scene)
        parent.layout().addWidget(self._image_view)

        qDebug("FfmpegView.__init__.")

    def decode_video(self, topic):
        # get bag from timeline
        bag = None
        self.start_stamp = self.timeline._get_start_stamp()
        start_time = self.start_stamp
        while bag is None:
            bag, entry = self.timeline.get_entry(start_time, topic)
            if bag is None:
                bag, entry = self.timeline.get_entry_after(start_time, topic)
                start_time = Time(nanoseconds=entry.timestamp)

        self.bag = bag

        # bag_helper.to_sec(self.timeline._get_start_stamp())
        # bag_helper.to_sec(self.bag.get_earliest_timestamp())
        # bag_helper.to_sec(self.bag.get_latest_timestamp())
        # bag_helper.to_sec(self.timeline._get_end_stamp())
        # self.start_stamp = self.timeline._get_start_stamp()
        # self.end_stamp = self.timeline._get_end_stamp()
        
        self.start_stamp = self.bag.get_earliest_timestamp()
        self.end_stamp = self.bag.get_latest_timestamp()

        # the current region-of-interest for our bag file
        # all resampling and plotting is done with these limits
        self.limits = [0, bag_helper.to_sec(self.end_stamp - self.start_stamp)]

        (ros_message, msg_type, topic) = self.bag.deserialize_entry(entry)


        # bag_entries= bag.get_entries_in_range(self.start_stamp + Duration(seconds=self.limits[0]),
        #                                      self.start_stamp + Duration(seconds=self.limits[1]),
        #                                      topic)
        bag_entries= bag.get_entries_in_range(self.start_stamp,
                                             self.end_stamp,
                                             topic)
        bagmsgs =[]
        for entry in bag_entries:
            # ros_message = deserialize_message(entry.data, msg_type)
            (ros_message, _, _) = self.bag.deserialize_entry(entry)
            bagmsgs.append((entry.topic, ros_message, entry.timestamp))

        self.video_decoder = BagVideoDecoder()
        self.video_decoder.load_packets_from_bagmsgs(bagmsgs)
        self._d_ht_idx = self.video_decoder._d_ht_idx

    # MessageView implementation
    def _resizeEvent(self, event):
        # TODO make this smarter. currently there will be no scrollbar even if the
        # timeline extends beyond the viewable area
        self.resize_signal.emit()

    def _resize_cb(self):
        qDebug("resize cb 00")
        with self.lock:
            self._scene.setSceneRect(
                0, 0, self._image_view.size().width() - 2, self._image_view.size().height() - 2)
            self.put_image_into_scene()
        qDebug("resize cb --")

    def message_viewed(self, *, entry, ros_message, msg_type_name, topic, **kwargs):
        """
        refreshes the image
        """
        qDebug("message_viewed 00")
        TopicMessageView.message_viewed(self, entry=entry)
        msg = ros_message, 
        if not msg:
            self.set_image(None, topic, 'no message')
        else:
            self.set_image(msg, topic, ros_message.header.stamp)
        qDebug("message_viewed --")

    def message_cleared(self):
        qDebug("message_cleared 00")
        TopicMessageView.message_cleared(self)
        self.set_image(None, None, None)
        qDebug("message_cleared --")

    # End MessageView implementation
    def put_image_into_scene(self):
        qDebug("put_image_into_scene 00")
        if self._image:
            scale_factor = min(
                float(self._image_view.size().width() - 2) / self._image.size[0],
                float(self._image_view.size().height() - 2) / self._image.size[1])
            resized_image = self._image.resize(
                (int(scale_factor * self._image.size[0]),
                 int(scale_factor * self._image.size[1])),
                self.quality)

            QtImage = ImageQt(resized_image)
            pixmap = QPixmap.fromImage(QtImage).copy()
            self._scene.clear()
            self._scene.addPixmap(pixmap)
        else:
            self._scene.clear()

    def set_image(self, image_msg, image_topic, image_stamp):
        qDebug("set_image 00")
        img = None
        if image_msg:
            img = self.video_decoder.get_pil_img(bag_helper.to_sec(Time.from_msg(image_stamp)))

        with self.lock:
            self._image_msg = image_msg

            if img is not None:
                self._image = img
            else:
                self._image = None

            self._image_topic = image_topic
            self._image_stamp = image_stamp
            self.put_image_into_scene()
