"""Read-only A+ admission observer. No reset, arm, grant or motion API."""
import math


class NeuPANFailstop:
    SCHEMAS = {'worker': 'r1_neupan_worker_health_v1',
               'controller': 'r1_neupan_execution_status_v1',
               'guard': 'r1_collision_guard_status_v1'}
    BOOTS = {'worker': 'boot_id', 'controller': 'controller_boot_id', 'guard': 'guard_boot_id'}

    def __init__(self, timeout=.5):
        self.timeout = timeout
        self.records = {}
        self.fault = ''
        self.worker_generation = None

    def observe(self, kind, row, now):
        if self.fault or not isinstance(row, dict):
            return
        try:
            if row.get('schema') != self.SCHEMAS[kind]:
                return  # diagnostics may share the controller topic
            boot = row[self.BOOTS[kind]]
            sequence, stamp = row['sequence'], row['monotonic']
            if (not isinstance(boot, str) or not boot or type(sequence) is not int or sequence < 1
                    or type(stamp) not in (float, int) or not math.isfinite(stamp)
                    or not 0 <= now - stamp <= self.timeout):
                return
            previous = self.records.get(kind)
            if previous:
                if previous['boot'] != boot:
                    self.fault = kind + '_boot_changed'
                    return
                if sequence <= previous['sequence']:
                    return
            if kind == 'worker':
                state = row['state']
                generation = row['worker_generation']
                if state not in ('STARTING', 'RUNNING', 'FAULT_LATCHED', 'SHUTTING_DOWN'):
                    return
                if type(generation) is not int or generation < -1:
                    return
                if self.worker_generation is not None and generation != self.worker_generation:
                    self.fault = 'worker_generation_changed'
                elif generation > 0:
                    self.worker_generation = generation
                if state == 'FAULT_LATCHED':
                    if not row.get('fault_id') or not row.get('fault_code'):
                        return
                    self.fault = 'worker_fatal:' + str(row['fault_code'])
            elif kind == 'controller':
                if type(row['fault_latched']) is not bool or type(row['plugin_active']) is not bool:
                    return
                if row['fault_latched']:
                    self.fault = 'controller_fatal:' + str(row.get('fault_reason', 'unknown'))
            else:
                if (row.get('authorization_protocol') != 'r1_neupan_motion_grant_v1'
                        or row.get('source_protocol') != 'r1_neupan_planner_source_v2'):
                    return
                if row.get('authorization_state') not in ('DISARMED', 'PREPARED', 'ARMED', 'SOURCE_STALE_HOLD', 'FAULT_LATCHED'):
                    return
                if row['authorization_state'] == 'FAULT_LATCHED':
                    self.fault = 'guard_fatal:' + str(row.get('authorization_reason', 'unknown'))
            self.records[kind] = dict(boot=boot, sequence=sequence, stamp=stamp, row=dict(row))
        except (KeyError, TypeError, ValueError):
            return

    def reason(self, now):
        if self.fault:
            return self.fault
        for kind in self.SCHEMAS:
            record = self.records.get(kind)
            if record is None or not 0 <= now - record['stamp'] <= self.timeout:
                return kind + '_status_missing_or_stale'
        if self.records['worker']['row']['state'] not in ('STARTING', 'RUNNING'):
            return 'worker_not_available'
        if not self.records['controller']['row']['plugin_active']:
            return 'controller_inactive'
        return ''
