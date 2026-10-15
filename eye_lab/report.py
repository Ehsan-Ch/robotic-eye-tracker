import argparse,json
from pathlib import Path

def build(out='reports'):
    out=Path(out);data=json.loads((out/'evaluation.json').read_text(encoding='utf-8'))
    replays={}
    for trial in data['trials']:
        s=trial['summary'];key=s['mode']+'_'+s['scenario']
        if key not in replays:replays[key]=trial
    payload={k:v for k,v in data.items() if k!='trials'};payload['replays']=replays
    template=Path(__file__).with_name('template.html').read_text(encoding='utf-8')
    (out/'report.html').write_text(template.replace('__DATA__',json.dumps(payload).replace('<','\\u003c')),encoding='utf-8',newline='\n')

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',default='reports');a=p.parse_args();build(a.out)
if __name__=='__main__':main()
