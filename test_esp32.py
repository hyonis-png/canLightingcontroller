"""Live CAN transport and reported lighting-state test, not an optical LED test.
Close Arduino Serial Monitor, then run: python test_esp32.py
"""
import argparse
from contextlib import ExitStack
from pathlib import Path
import re
import sys
import time
import serial

COMMANDS = {'0':16, '1':15, '2':9, '3':10, '4':11, '5':12}
FRAME = re.compile(r'RX CAN ID: 0x([0-9A-Fa-f]+) DLC: (\d+) DATA:\s*(.*?)\s*$')

def next_state(state, command):
    if command in {'1','2','4'}:
        return {'1':'HAZARD_ON','2':'LEFT_ON','4':'RIGHT_ON'}[command]
    target = {'0':'HAZARD_ON','3':'LEFT_ON','5':'RIGHT_ON'}[command]
    return 'ALL_OFF' if state == target else state

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--esp', default='COM6')
    parser.add_argument('--mega', default='COM5')
    parser.add_argument('--rounds', type=int, default=5)
    parser.add_argument('--log', default='can_test.log')
    args = parser.parse_args()
    if args.rounds < 1:
        parser.error('--rounds must be positive')
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(errors='replace')
    with ExitStack() as stack:
        log = stack.enter_context(Path(args.log).open('w', encoding='utf-8'))
        esp = stack.enter_context(serial.Serial(args.esp,9600,timeout=0,write_timeout=2))
        mega = stack.enter_context(serial.Serial(args.mega,9600,timeout=0,write_timeout=2))
        ports = {'ESP':esp, 'MEGA':mega}
        buffers = {label:b'' for label in ports}
        def record(line):
            print(line, flush=True)
            log.write(line+'\n'); log.flush()
        def collect(seconds):
            result=[]
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                for label,p in ports.items():
                    buffers[label]+=p.read(p.in_waiting)
                    while b'\n' in buffers[label]:
                        raw,buffers[label]=buffers[label].split(b'\n',1)
                        line=raw.decode(errors='replace').strip()
                        record(f'{label} {line}')
                        result.append((label,line))
                time.sleep(.005)
            return result
        collect(4)
        # Establish a known state without discarding startup evidence.
        for command in '035':
            esp.write(command.encode()); collect(.8)
        state='ALL_OFF'
        passed=0
        # Includes repeated ON and OFF commands that preserve another mode.
        sequence='113024523405'
        for index,command in enumerate(sequence*args.rounds,1):
            collect(.1)
            esp.write(command.encode())
            rows=collect(1.0)
            state=next_state(state,command)
            frames=[]
            for label,line in rows:
                match=FRAME.fullmatch(line) if label=='MEGA' else None
                if match:
                    frames.append((int(match[1],16),int(match[2]),match[3].split()))
            expected=(0x7FF,1,[str(COMMANDS[command])])
            ok=(frames==[expected] and ('ESP','TX_COMPLETE') in rows
                and ('MEGA',f'STATE: {state}') in rows)
            record(f'TEST {index}: {"PASS" if ok else "FAIL"} command={command} frame={expected} state={state}')
            if not ok:
                for p in ports.values(): p.write(b'?')
                collect(.5)
                record('RESULT: FAIL (stopped at first failing command)')
                return 1
            passed+=1
        for p in ports.values(): p.write(b'?')
        rows=collect(.5)
        clean=set()
        for label,line in rows:
            if line.startswith('STATUS '):
                fields=dict(re.findall(r'(\w+)=([0-9A-F]+)',line))
                if all(fields.get(k)=='0' for k in ('EFLG','TEC','REC')):
                    clean.add(label)
        if clean != {'ESP','MEGA'}:
            record('RESULT: FAIL (missing status or nonzero error flags/counters)')
            return 1
        record(f'RESULT: PASS {passed}/{passed} frames and reported lighting states; both error counters zero.')
        record('Headlights/brakes are not implemented by this Mega sketch and are not lighting-state tests.')
        return 0

if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (serial.SerialException,OSError) as exc:
        print(f'Hardware test could not complete: {exc}',file=sys.stderr)
        raise SystemExit(2)
