"""Bounded exact-time association; no interpolation or measurement retiming."""
import math


class ExactImagePing:
    def __init__(self):
        self.images={};self.pings={};self.last_image=0;self.last_ping=0;self.released=0
        self.matched=0;self.unpaired=0;self.late=0

    def image(self,stamp,payload,now):
        if type(stamp) is not int or stamp<=self.last_image:raise ValueError('image_pair_clock')
        if not math.isfinite(now):raise ValueError('image_pair_wall')
        if len(self.images)>=4:raise ValueError('image_pair_backlog')
        self.last_image=stamp;self.images[stamp]=(payload,now)

    def ping(self,stamp,value):
        if type(stamp) is not int or stamp<=self.last_ping:raise ValueError('ping_pair_clock')
        if value is not None and (not math.isfinite(value) or value<=0):raise ValueError('ping_pair_range')
        self.last_ping=stamp
        if stamp<=self.released:self.late+=1;return
        if len(self.pings)>=64:raise ValueError('ping_pair_backlog')
        self.pings[stamp]=value

    def drain(self,now):
        result=[]
        for stamp,(payload,arrival) in list(self.images.items()):
            if stamp not in self.pings and now-arrival<.15:break
            value=self.pings.get(stamp)
            if value is None:self.unpaired+=1
            else:self.matched+=1
            result.append((payload,None if value is None else (stamp,value)))
            del self.images[stamp];self.released=stamp
            self.pings={t:v for t,v in self.pings.items() if t>stamp}
        return result
