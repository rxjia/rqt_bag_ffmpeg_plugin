import av

class BagVideoDecoder():
    def __init__(self, cache_all=False):
        self.imgs = []
        self._d_ht_idx = {}
        self.cache_kf_id = None
        self.d_kf_id_off = {}  # key: float stamp,  value: (kf_id, id_offset)
        self._codec = av.CodecContext.create("h264", "r")
        self._cache_all = cache_all
        self._id_kf_last = -1
        self._id_max_offset = -1
        self._local_frames = []

    def __del__(self):
        if self._codec.is_open:
            self._codec.close()

    def load_packets_from_bagmsgs(self, bagmsgs):
        d_ht_idx = {}
        d_kf_id_off = {}
        l_frame = []
        l_packet = []

        first_kf_id = -1
        msg_idx = 0
        kf_id = -1
        for topic, msg, ts in bagmsgs:
            if msg.flags == 1:
                kf_id = msg_idx
            if first_kf_id == -1:
                if msg.flags == 1:
                    first_kf_id = msg_idx

            if first_kf_id != -1:
                packet = av.packet.Packet(msg.data)
                packet.pts = msg.pts
                packet.dts = msg.pts
                l_packet.append(packet)
                if self._cache_all:
                    frames = self._codec.decode(packet)
                    for frame in frames:
                        l_frame.append(frame)
            else:
                if self._cache_all:
                    l_frame.append(None)
                l_packet.append(None)

            key = msg.header.stamp.to_sec()
            d_ht_idx[key] = msg_idx
            d_kf_id_off[key] = (kf_id, msg_idx - kf_id)
            msg_idx += 1

        self.imgs = l_frame
        self._packets = l_packet
        self._d_ht_idx = d_ht_idx
        # self._l_ht = np.array(list(d_ht_idx.keys()))
        self.d_kf_id_off = d_kf_id_off
        self._l_kf_id_off = list(d_kf_id_off.values())
        self.first_kd_id = first_kf_id

    def get_pil_img(self, ht):
        return self.get_pil_img_at_t(ht, max_diff=0)

    def get_pil_img_at_t(self, t, max_diff=0):
        """
        :param t: float, time stamp
        :param max_diff: float, max time difference between t and the nearest time stamp
        :return: PIL image
        """
        if max_diff == 0:
            idx = self._d_ht_idx.get(t)
            # print(f"img_idx: {idx}")
            
            if not(idx and idx < len(self._l_kf_id_off)):
                return None

        if self._cache_all:
            return self.imgs[idx].to_image()
        else:
            # ht = self._l_ht[idx]
            # id_kf, id_off = self.d_kf_id_off[ht]
            id_kf, id_off = self._l_kf_id_off[idx]
            if id_kf == -1:
                print(f'no key frame found at {t}')
                return None

            if id_kf == self._id_kf_last:
                if id_off <= self._id_max_offset:
                    return self._local_frames[id_off].to_image()
                else:
                    for i in range(id_kf + self._id_max_offset + 1, id_kf + id_off + 1):
                        frames = self._codec.decode(self._packets[i])
                        for frame in frames:
                            self._local_frames.append(frame)

                    self._id_max_offset = max(self._id_max_offset, id_off)
                    return self._local_frames[-1].to_image()
            else:
                if self._codec.is_open:
                    self._codec.close()
                self._codec = av.CodecContext.create("h264", "r")

                self._local_frames = []
                for i in range(id_kf, id_kf + id_off+1):
                    frames = self._codec.decode(self._packets[i])
                    for frame in frames:
                        self._local_frames.append(frame)

                self._id_kf_last = id_kf
                self._id_max_offset = id_off
                return self._local_frames[-1].to_image()