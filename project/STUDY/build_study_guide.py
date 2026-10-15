#!/usr/bin/env python3
"""Build 'Project Study Guide.html' from study_guide_template.html.

Code blocks are cut out of the real files at build time, and the results are graded from the
bags at build time, so the guide cannot drift from the code or the data.

    source /opt/ros/jazzy/setup.bash
    python3 build_study_guide.py
"""

import base64
import datetime
import html
import io
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.join(HERE, '..')
PKG = os.path.join(PROJECT, 'ros2_ws', 'src', 'r7021e_project')
BAGS = os.path.join(PROJECT, 'bags')
PLOTS = os.path.join(PROJECT, 'plots')

FILES = {
    'mpc_node.py': os.path.join(PKG, 'r7021e_project', 'mpc_node.py'),
    'project_sim.launch.py': os.path.join(PKG, 'launch', 'project_sim.launch.py'),
    'burger_plank.sdf': os.path.join(PKG, 'worlds', 'burger_plank.sdf'),
    'plot_run.py': os.path.join(PROJECT, 'analysis', 'plot_run.py'),
}

sys.path.insert(0, os.path.join(PROJECT, 'analysis'))
import plot_run   # noqa: E402  (the grader; imports the node's tables)


def norm(line):
    return re.sub(r'-{3,}', '---', line)


def cut(name, start, end):
    """Lines of FILES[name] from the first line containing start up to (not including) the
    first later line containing end."""
    lines = open(FILES[name]).read().split('\n')
    i = next(k for k, l in enumerate(lines) if norm(start) in norm(l))
    j = next(k for k in range(i + 1, len(lines)) if norm(end) in norm(lines[k]))
    block = lines[i:j]
    while block and not block[-1].strip():
        block.pop()
    return '\n'.join(block), i + 1


def code_block(match):
    name, start, end = match.group(1), match.group(2), match.group(3)
    text, first = cut(name, start, end)
    return ('<div class="codehead">%s, from line %d</div><pre class="code">%s</pre>'
            % (name, first, html.escape(text)))


def png(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=130)
    plt.close(fig)
    return 'data:image/png;base64,' + base64.b64encode(buf.getvalue()).decode()


def file_png(path):
    return 'data:image/png;base64,' + base64.b64encode(open(path, 'rb').read()).decode()


def figure(src, caption):
    return '<figure><img src="%s" alt=""><figcaption>%s</figcaption></figure>' % (src, caption)


def grade(bag, scenario):
    path = os.path.join(BAGS, bag)
    if not os.path.isdir(path) or not os.path.exists(os.path.join(path, 'metadata.yaml')):
        return None   # missing, or still being recorded
    return plot_run.grade(path, scenario, bag.split('_')[-1])


def pair_plot(scenario):
    a, b = grade('%s_one' % scenario, scenario), grade('%s_three' % scenario, scenario)
    if not a or not b:
        return None, a, b
    out = os.path.join(PLOTS, '%s_pair.png' % scenario)
    plot_run.plot([a, b], out)
    return file_png(out), a, b


def ms(res, k):
    return '%.0f' % (res['solve'][k] * 1e3) if res and 'solve' in res else '–'


def results_html():
    parts = []
    mc = plot_run.MC

    # Scenario 2 sweep
    rows = []
    sweep = []
    for gap in mc.GAPS:
        sc = 's2_%03d' % round(gap * 100)
        one, three = grade(sc + '_one', sc), grade(sc + '_three', sc)
        sweep.append((gap, one, three))

        def cell(r):
            if r is None:
                return '<td colspan="3">not run</td>'
            outcome = ('reached %.1f s' % r['reached']) if r['reached'] is not None else ('stopped %.2f m short' % r['final_error'])
            cls = 'pass' if plot_run.passed(r) else 'fail'
            return ('<td class="%s">%s</td><td>%s</td><td>%+.3f</td>'
                    % (cls, 'PASS' if cls == 'pass' else 'FAIL', outcome, min(r['clearance'])))

        rows.append('<tr><td>%.2f</td>%s%s<td>%s / %s</td><td>%s / %s</td></tr>'
                    % (gap, cell(one), cell(three), ms(one, 0), ms(three, 0), ms(one, 1), ms(three, 1)))

    parts.append('<h3>Scenario 2: the gap sweep (Gazebo, ground truth)</h3>')
    parts.append('<div class="tablewrap"><table><tr><th rowspan="2">gap (m)</th><th colspan="3">one disc, r 0.269</th>'
                 '<th colspan="3">three discs, r 0.130</th><th rowspan="2">solve median ms<br>one / three</th>'
                 '<th rowspan="2">solve p95 ms<br>one / three</th></tr>'
                 '<tr><th></th><th>outcome</th><th>min real clearance (m)</th><th></th><th>outcome</th>'
                 '<th>min real clearance (m)</th></tr>' + ''.join(rows) + '</table></div>')

    passed_one = [g for g, o, t in sweep if o and plot_run.passed(o)]
    passed_three = [g for g, o, t in sweep if t and plot_run.passed(t)]
    parts.append('<div class="box result">Narrowest gap passed: <b>one disc %s</b>, <b>three discs %s</b>. '
                 'The real plank is 0.20 m wide.</div>'
                 % ('%.2f m' % min(passed_one) if passed_one else 'none',
                    '%.2f m' % min(passed_three) if passed_three else 'none'))

    # Sweep chart
    fig, ax = plt.subplots(figsize=(8, 4.2))
    for idx, (label, colour, pick) in enumerate((('one disc', 'tab:red', 1), ('three discs', 'tab:blue', 2))):
        gx = [s[0] for s in sweep if s[pick]]
        cl = [min(s[pick]['clearance']) for s in sweep if s[pick]]
        ok = [plot_run.passed(s[pick]) for s in sweep if s[pick]]
        ax.plot(gx, cl, '-', color=colour, alpha=0.4)
        ax.scatter(gx, cl, color=colour, marker='o', s=60, label=label + ' (filled = pass)',
                   facecolors=[colour if o else 'white' for o in ok], zorder=3)
    ax.axvline(0.539, color='tab:red', ls=':', lw=1)
    ax.axvline(0.260, color='tab:blue', ls=':', lw=1)
    ax.axvline(0.200, color='0.3', ls='--', lw=1)
    ax.text(0.541, ax.get_ylim()[1] * 0.92, 'one disc needs 0.539', color='tab:red', fontsize=8)
    ax.text(0.262, ax.get_ylim()[1] * 0.80, 'three need 0.260', color='tab:blue', fontsize=8)
    ax.text(0.202, ax.get_ylim()[1] * 0.68, 'plank 0.20', color='0.3', fontsize=8)
    ax.set_xlabel('gap between post surfaces [m]')
    ax.set_ylabel('min real plank clearance [m]')
    ax.set_xlim(0.18, 0.64)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc='upper left')
    ax.set_title('Scenario 2: hollow = rejected the gap (stopped), filled = passed')
    fig.tight_layout()
    parts.append(figure(png(fig), 'Every point is one Gazebo run graded from ground truth. A one-disc run that stops '
                        'in front of the posts still has positive clearance: it was safe, it just never went through.'))

    for sc in ('s2_050', 's2_028'):
        src, _, _ = pair_plot(sc)
        if src:
            parts.append(figure(src, '%s: red one disc, blue three discs. Plank outlines drawn from ground truth.' % sc))

    # Scenario 1
    parts.append('<h3>Scenario 1: known path with an obstacle (Gazebo, ground truth)</h3>')
    rows = []
    figs = []
    for sc, what in (('s1a', 'circle r 0.8, obstacle (0, 0.62) r 0.12'), ('s1b', 'figure-8, obstacle (0, 0.36) r 0.08')):
        src, one, three = pair_plot(sc)
        for r in (one, three):
            if r is None:
                continue
            rows.append('<tr><td>%s</td><td>%s</td><td>%.4f</td><td>%.4f</td><td>%+.4f</td><td>%s</td><td>%s</td></tr>'
                        % (sc, r['footprint'], r['path_error_mean'], r['path_error_max'], min(r['clearance']),
                           ms(r, 0), ms(r, 1)))
        if src:
            figs.append(figure(src, '%s, %s. Red one disc, blue three discs.' % (sc, what)))
    parts.append('<div class="tablewrap"><table><tr><th>scenario</th><th>footprint</th><th>path error mean (m)</th>'
                 '<th>path error max (m)</th><th>min real clearance (m)</th><th>solve median ms</th><th>solve p95 ms</th></tr>'
                 + ''.join(rows) + '</table></div>')
    parts.append('{{S1_TEXT}}')
    parts.extend(figs)

    # Solve time overall
    med = {'one': [], 'three': []}
    for name in sorted(os.listdir(BAGS)):
        fp = name.split('_')[-1]
        sc = '_'.join(name.split('_')[:-1])
        if fp in med and sc in mc.SCENARIOS:
            r = grade(name, sc)
            if r and 'solve' in r:
                med[fp].append(r['solve'][0] * 1e3)
    if med['one'] and med['three']:
        parts.append('<h3>Solve time per tick, all Gazebo runs (GUI off)</h3>')
        parts.append('<div class="box result">Median of the per-run medians: one disc <b>%.0f ms</b>, three discs '
                     '<b>%.0f ms</b>, a factor of %.1f. The tick is 100 ms.</div>'
                     % (np.median(med['one']), np.median(med['three']), np.median(med['three']) / np.median(med['one'])))

    parts.append('{{RESULTS_TEXT}}')
    return '\n'.join(parts)


def main():
    template = open(os.path.join(HERE, 'study_guide_template.html')).read()
    theory = open(os.path.join(HERE, 'Project Theory Guide.html')).read()
    style = re.search(r'<style>.*?</style>', theory, re.S).group(0)

    out = template.replace('{{STYLE}}', style)
    out = re.sub(r'\{\{code\|([^|]+)\|([^|]+)\|([^}]+)\}\}', code_block, out)
    out = out.replace('{{RESULTS}}', results_html())

    notes = os.path.join(HERE, 'study_guide_results.html')
    texts = open(notes).read() if os.path.exists(notes) else ''
    for key in ('S1_TEXT', 'RESULTS_TEXT'):
        m = re.search(r'<!--%s-->(.*?)<!--/%s-->' % (key, key), texts, re.S)
        out = out.replace('{{%s}}' % key, m.group(1) if m else '')

    out = re.sub(r'\{\{img\|([^|]+)\|([^}]+)\}\}',
                 lambda m: figure(file_png(os.path.join(PROJECT, m.group(1))), m.group(2)), out)
    out = out.replace('{{DATE}}', datetime.date.today().isoformat())

    dst = os.path.join(HERE, 'Project Study Guide.html')
    open(dst, 'w').write(out)
    print('wrote %s (%d kB)' % (dst, len(out) // 1024))


if __name__ == '__main__':
    main()
