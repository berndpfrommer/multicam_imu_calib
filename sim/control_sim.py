import subprocess
import time


def make_cmd(cmd, val):
    return ['gz', 'topic', '-t', f'/{cmd}_position', '-m', 'gz.msgs.Double', '-p', f'data: {val}']


if __name__ == '__main__':
    cmds = (
        {'cmd': 'z_rot', 'limits': (0.8, -0.8), 'wait': 2.0},
        {'cmd': 'y_rot', 'limits': (0.35, -0.35), 'wait': 2.0},
        {'cmd': 'x_rot', 'limits': (1.570796327, -1.570796327), 'wait': 2.0},
        {'cmd': 'z_trans', 'limits': (0.5, -0.5), 'wait': 2.0},
        {'cmd': 'y_trans', 'limits': (-0.5, 0.5), 'wait': 2.0},
        {'cmd': 'x_trans', 'limits': (-0.5, 0.5), 'wait': 2.0},
    )
    for i, c in enumerate(cmds):
        print(f'move: {i}')
        a = make_cmd(c['cmd'], c['limits'][0])
        print(a)
        subprocess.run(a)
        time.sleep(c['wait'])
        a = make_cmd(c['cmd'], c['limits'][1])
        print(a)
        subprocess.run(a)
        time.sleep(c['wait'])
        a = make_cmd(c['cmd'], 0)
        print(a)
        subprocess.run(a)
        time.sleep(c['wait'])
    print('sequence complete!')
