"""Controller feedback guard; experiment observer is never a control fallback."""
import math


def zero_command_stamp(last_control_stamp,observer):
    """Only for zero commands: never send a timestamp older than prior tail zeros."""
    if observer and observer.get('valid'):
        stamp=observer['solver_state_ned_frd']['t']
        if math.isfinite(stamp): return max(last_control_stamp,stamp)
    return last_control_stamp

class FeedbackGuard:
    def __init__(self):
        self.latest=None
        self.received=None
        self.reason=None

    def trip(self,reason):
        if self.reason is None: self.reason=reason

    def receive(self,record,now):
        if self.reason: return
        if not record.get('valid'):
            self.trip('invalid_state'); return
        stamp=record['solver_state_ned_frd']['t']
        if not math.isfinite(stamp):
            self.trip('invalid_state'); return
        if self.latest is not None:
            old=self.latest['solver_state_ned_frd']['t']
            if stamp<=old or record['sequence']<=self.latest['sequence']:
                self.trip('stale_state'); return
            if record['sequence']!=self.latest['sequence']+1 or abs(stamp-old-.05)>1e-6:
                self.trip('state_gap'); return
        self.latest=record; self.received=now

    def check_timeout(self,now):
        if self.received is not None and now-self.received>.25:
            self.trip('state_timeout')
        return self.reason
