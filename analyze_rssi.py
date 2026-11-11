#!/usr/bin/env python3
"""
analyze_rssi.py - xu ly CSV do rssi_sim (hoac wifi_survey that) thanh heatmap.

CSV can cot: x, y, rssi_dbm   (tuy chon: rssi_mean_dbm = dap an dung cua mo phong)

Buoc xu ly (khop slide 5.4):
  1. chia map thanh o cell x cell (mac dinh 0.5 m)
  2. moi o: loc ngoai lai bang MAD, lay median
  3. noi suy IDW ra luoi min, chi noi suy gan diem do (khong ngoai suy xa)
  4. danh gia: leave-one-out RMSE; neu co dap an dung thi RMSE so voi dap an
  5. hoi quy path loss RSSI = P0 - 10 n log10(d) neu biet vi tri AP
  6. ti le dien tich dat nguong (mac dinh -67 dBm)

Chay:
  python3 analyze_rssi.py /tmp/rssi_log.csv --ap 1.0 2.0 --out result
"""
import argparse
import csv
import os

import numpy as np
from scipy.spatial import cKDTree


def load(path):
    with open(path) as f:
        rd = csv.DictReader(f)
        rows = list(rd)
    need = ['x', 'y', 'rssi_dbm']
    for k in need:
        if k not in rows[0]:
            raise SystemExit(f'CSV thieu cot {k}. Cot hien co: {list(rows[0].keys())}')
    x = np.array([float(r['x']) for r in rows])
    y = np.array([float(r['y']) for r in rows])
    v = np.array([float(r['rssi_dbm']) for r in rows])
    t = np.array([float(r['rssi_mean_dbm']) for r in rows]) if 'rssi_mean_dbm' in rows[0] else None
    return x, y, v, t


def mad_median(vals, k=3.0, floor_db=1.0):
    """Loc ngoai lai bang MAD roi lay median. Tra ve (median, so mau giu lai, so mau bo)."""
    med = np.median(vals)
    mad = max(np.median(np.abs(vals - med)) * 1.4826, floor_db)
    keep = np.abs(vals - med) <= k * mad
    kept = vals[keep]
    return float(np.median(kept)), int(keep.sum()), int((~keep).sum())


def aggregate(x, y, v, t, cell, min_n):
    x0, y0 = x.min(), y.min()
    ix = np.floor((x - x0) / cell).astype(int)
    iy = np.floor((y - y0) / cell).astype(int)
    cells = {}
    for i, (a, b) in enumerate(zip(ix, iy)):
        cells.setdefault((a, b), []).append(i)
    cx, cy, cv, ct, cn = [], [], [], [], []
    dropped = 0
    for (a, b), idx in cells.items():
        if len(idx) < min_n:
            continue
        med, nk, nb = mad_median(v[idx])
        dropped += nb
        cx.append(x0 + (a + 0.5) * cell)
        cy.append(y0 + (b + 0.5) * cell)
        cv.append(med)
        cn.append(nk)
        ct.append(float(np.mean(t[idx])) if t is not None else np.nan)
    return (np.array(cx), np.array(cy), np.array(cv), np.array(ct), np.array(cn),
            dropped, len(cells))


def idw(px, py, pv, qx, qy, k=8, power=2.0, max_dist=2.0):
    tree = cKDTree(np.c_[px, py])
    kk = min(k, len(px))
    d, i = tree.query(np.c_[qx, qy], k=kk)
    d = np.atleast_2d(d.T).T if kk == 1 else d
    i = np.atleast_2d(i.T).T if kk == 1 else i
    d = d.reshape(len(qx), kk)
    i = i.reshape(len(qx), kk)
    w = 1.0 / np.maximum(d, 1e-6) ** power
    out = np.sum(w * pv[i], axis=1) / np.sum(w, axis=1)
    out[d[:, 0] > max_dist] = np.nan          # khong ngoai suy xa
    return out


def loo_rmse(px, py, pv, k, power):
    errs = []
    for j in range(len(px)):
        m = np.ones(len(px), bool)
        m[j] = False
        p = idw(px[m], py[m], pv[m], px[j:j + 1], py[j:j + 1], k, power, max_dist=np.inf)
        errs.append(p[0] - pv[j])
    errs = np.array(errs)
    return float(np.sqrt(np.mean(errs ** 2))), errs


def load_free_mask(yaml_path):
    """Doc map_server yaml + pgm. Tra ve ham free(x, y) -> bool (o trong thuc su)."""
    import yaml
    from matplotlib import image as mpimg
    y = yaml.safe_load(open(yaml_path))
    img_path = y['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    img = mpimg.imread(img_path)
    if img.ndim == 3:
        img = img[..., 0]
    img = img.astype(float)
    if img.max() <= 1.0:
        img = img * 255.0
    res = float(y['resolution'])
    ox, oy = float(y['origin'][0]), float(y['origin'][1])
    negate = int(y.get('negate', 0))
    p = img / 255.0 if negate else (255.0 - img) / 255.0     # xac suat chiem
    free = p < float(y.get('free_thresh', 0.25))
    H, W = free.shape

    def inside(x, yy):
        c = np.floor((np.asarray(x) - ox) / res).astype(int)
        r = H - 1 - np.floor((np.asarray(yy) - oy) / res).astype(int)
        ok = (r >= 0) & (r < H) & (c >= 0) & (c < W)
        out = np.zeros(np.shape(x), bool)
        out[ok] = free[r[ok], c[ok]]
        return out
    return inside


def fit_pathloss(px, py, pv, ap):
    d = np.hypot(px - ap[0], py - ap[1])
    m = d >= 1.0
    A = np.c_[np.ones(m.sum()), -10 * np.log10(d[m])]
    (p0, n), *_ = np.linalg.lstsq(A, pv[m], rcond=None)
    res = pv[m] - A @ np.array([p0, n])
    return float(p0), float(n), float(res.std()), int(m.sum())


def main():
    ap_ = argparse.ArgumentParser()
    ap_.add_argument('csv')
    ap_.add_argument('--cell', type=float, default=0.5)
    ap_.add_argument('--min-n', type=int, default=3, help='so mau toi thieu moi o')
    ap_.add_argument('--grid', type=float, default=0.25, help='buoc luoi heatmap (m)')
    ap_.add_argument('--k', type=int, default=8)
    ap_.add_argument('--power', type=float, default=2.0)
    ap_.add_argument('--max-dist', type=float, default=2.0)
    ap_.add_argument('--thr', type=float, default=-67.0)
    ap_.add_argument('--ap', type=float, nargs=2, default=None, metavar=('X', 'Y'))
    ap_.add_argument('--map', default=None, help='map_server yaml (vd tb3_house.yaml): '
                     'chi tinh dien tich tren khong gian trong thuc su')
    ap_.add_argument('--truth', default=None, help='file dap an dung do rssi_sim ghi ra '
                     '(-p truth_csv:=...), de so sanh heatmap voi dap an')
    ap_.add_argument('--out', default='result')
    a = ap_.parse_args()
    os.makedirs(a.out, exist_ok=True)

    x, y, v, t = load(a.csv)
    ok = np.isfinite(v) & (v > -120) & (v < 0)       # bo mau loi
    x, y, v = x[ok], y[ok], v[ok]
    t = t[ok] if t is not None else None
    print(f'Doc {len(x)} mau ({(~ok).sum()} mau loi bi bo)')

    cx, cy, cv, ct, cn, dropped, ncell_all = aggregate(x, y, v, t, a.cell, a.min_n)
    print(f'{ncell_all} o co du lieu, {len(cx)} o du >= {a.min_n} mau; '
          f'MAD loai {dropped} mau ngoai lai')

    rm, errs = loo_rmse(cx, cy, cv, a.k, a.power)
    print(f'Leave-one-out RMSE (IDW, tren {len(cx)} o): {rm:.2f} dB')

    if t is not None and np.isfinite(ct).any():
        e = cv - ct
        print(f'So voi dap an dung (trung binh khong nhieu): RMSE o median = '
              f'{np.sqrt(np.mean(e**2)):.2f} dB, lech TB = {e.mean():+.2f} dB')

    if a.ap:
        p0, n, sd, m = fit_pathloss(cx, cy, cv, a.ap)
        print(f'Path loss (tren {m} o, d>=1 m): P0 = {p0:.1f} dBm, n = {n:.2f}, '
              f'do lech du = {sd:.2f} dB')

    gx = np.arange(cx.min() - a.max_dist, cx.max() + a.max_dist + a.grid, a.grid)
    gy = np.arange(cy.min() - a.max_dist, cy.max() + a.max_dist + a.grid, a.grid)
    GX, GY = np.meshgrid(gx, gy)
    Z = idw(cx, cy, cv, GX.ravel(), GY.ravel(), a.k, a.power, a.max_dist).reshape(GX.shape)
    if a.map:
        free = load_free_mask(a.map)(GX, GY)
        total_free = free.sum()
        Z = np.where(free, Z, np.nan)               # bo phan nam trong tuong
    valid = np.isfinite(Z)
    frac = float(np.mean(Z[valid] >= a.thr)) * 100
    print(f'Dien tich dat RSSI >= {a.thr:.0f} dBm: {frac:.1f}% cua vung co du lieu '
          f'({valid.sum()} o luoi, {valid.sum()*a.grid**2:.1f} m2)')
    if a.map:
        print(f'Do phu du lieu: {valid.sum()/total_free*100:.1f}% khong gian trong cua map '
              f'({total_free*a.grid**2:.1f} m2) da co so do/noi suy; phan con lai la chua do')

    if a.truth:
        T = np.loadtxt(a.truth, delimiter=',', skiprows=1)
        tx, ty, tv = T[:, 0], T[:, 1], T[:, 2]
        if a.map:
            keep = load_free_mask(a.map)(tx, ty)
            tx, ty, tv = tx[keep], ty[keep], tv[keep]
        ux = np.unique(np.round(tx, 3))
        step = float(np.min(np.diff(ux))) if len(ux) > 1 else a.grid
        dmin, _ = cKDTree(np.c_[cx, cy]).query(np.c_[tx, ty])
        meas = dmin <= a.max_dist
        pred = idw(cx, cy, cv, tx[meas], ty[meas], a.k, a.power, a.max_dist)
        err = pred - tv[meas]
        A = step * step
        print('--- SO VOI DAP AN DUNG (cung mot vung, tinh tren o trong cua map) ---')
        print(f'Tong khong gian trong: {len(tx)*A:.1f} m2; da do/noi suy: '
              f'{meas.sum()*A:.1f} m2 ({meas.mean()*100:.1f}%); chua do: {(~meas).sum()*A:.1f} m2')
        print(f'Heatmap IDW so voi dap an tren vung da do: RMSE = '
              f'{np.sqrt(np.mean(err**2)):.2f} dB, lech TB = {err.mean():+.2f} dB')
        print(f'Ti le dat >= {a.thr:.0f} dBm theo DAP AN: vung da do {np.mean(tv[meas]>=a.thr)*100:.1f}%'
              f' | vung chua do ' + (f'{np.mean(tv[~meas]>=a.thr)*100:.1f}% (RSSI thap nhat {tv[~meas].min():.1f} dBm)' if (~meas).any() else 'khong co') +
              f' | toan bo {np.mean(tv>=a.thr)*100:.1f}%')
        print(f'Ti le dat >= {a.thr:.0f} dBm theo HEATMAP tren vung da do: '
              f'{np.mean(pred>=a.thr)*100:.1f}%')

    np.savetxt(os.path.join(a.out, 'cells.csv'), np.c_[cx, cy, cv, cn],
               delimiter=',', header='x,y,rssi_median_dbm,n_samples', comments='', fmt='%.3f')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    sc = axs[0].scatter(x, y, c=v, s=6, cmap='viridis', vmin=-90, vmax=-30)
    axs[0].set_title(f'Mau tho ({len(x)} mau)')
    im = axs[1].pcolormesh(GX, GY, np.ma.masked_invalid(Z), cmap='viridis',
                           vmin=-90, vmax=-30, shading='auto')
    axs[1].scatter(cx, cy, c='white', s=4, edgecolors='k', linewidths=0.2)
    if valid.any() and np.nanmin(Z) < a.thr < np.nanmax(Z):
        axs[1].contour(GX, GY, np.ma.masked_invalid(Z), levels=[a.thr], colors='r',
                       linewidths=1.2)
    axs[1].set_title(f'IDW heatmap, o {a.cell} m, do do {a.thr:.0f} dBm (do)')
    if a.ap:
        for ax in axs:
            ax.plot(*a.ap, 'r^', ms=9)
    for ax in axs:
        ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    fig.colorbar(im, ax=axs, label='RSSI (dBm)')
    out = os.path.join(a.out, 'heatmap.png')
    fig.savefig(out, dpi=150)
    print('Da luu', out, 'va', os.path.join(a.out, 'cells.csv'))


if __name__ == '__main__':
    main()
