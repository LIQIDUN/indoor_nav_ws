"""Publication outage accounting; diagnostics only, never relaxes health gates."""


class PublicationTracker:
    def __init__(self, expected_period=None):
        self.expected_period = expected_period
        self.last_success = None
        self.blocked_since = None
        self.reason = None
        self.last_tick = None
        self.sequence = 0

    def update(self, now, sent, reason, context):
        events = []
        tick_gap = None if self.last_tick is None else now-self.last_tick
        if self.last_tick is not None and now < self.last_tick:
            events.append(dict(context, event='TIME_RESET', stamp_s=now,
                               previous_tick_s=self.last_tick))
            self.blocked_since = None
            self.last_success = None
            self.reason = None
        if (tick_gap is not None and self.expected_period is not None and
                tick_gap > 1.5*self.expected_period):
            events.append(dict(context,event='TICK_GAP',stamp_s=now,
                reason='PUBLICATION_CALLBACK_GAP',tick_gap_s=tick_gap,
                estimated_missing_since_s=self.last_tick+self.expected_period,
                last_success_s=self.last_success,
                timing_precision='observed on next callback; start is estimated'))
        if sent:
            if self.blocked_since is not None:
                events.append(dict(context,event='RESUMED',stamp_s=now,
                    stopped_since_s=self.blocked_since,duration_s=now-self.blocked_since,
                    previous_reason=self.reason,last_success_before_gap_s=self.last_success))
            self.last_success = now
            self.blocked_since = None
            self.reason = None
        else:
            if self.blocked_since is None:
                self.blocked_since = now
                kind = 'STOPPED'
            else:
                kind = 'REASON_CHANGED'
            if self.reason != reason:
                events.append(dict(context,event=kind,stamp_s=now,reason=reason,
                    stopped_since_s=self.blocked_since,last_success_s=self.last_success))
            self.reason = reason
        self.last_tick = now
        for event in events:
            self.sequence += 1
            event['sequence'] = self.sequence
        return {'sent_this_tick':sent,'reason':reason,'last_success_s':self.last_success,
                'stopped_since_s':self.blocked_since,
                'outage_duration_s':now-self.blocked_since if self.blocked_since is not None else 0.,
                'tick_gap_s':tick_gap}, events
