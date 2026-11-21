"""Vision de l'échiquier : photo -> état du plateau. Sans ROS (utilisé par chess_perception_node).

1. Plateau : findChessboardCorners échoue (les pièces cachent les coins). On détecte les cases
   noires visibles, on cherche par RANSAC la plus grande grille de damier cohérente, puis on la
   place dans le 8x8 en comparant les couleurs attendues (damier dedans, papier blanc autour).
   -> homographie image -> repère plateau (mètres).
2. Pièces : boîtes YOLO, doublons supprimés (même pièce vue en reine et en roi, tête du cavalier
   vue comme un pion, une seule pièce par case). vote() fusionne plusieurs images. Le bas de la boîte est projeté sur le plateau puis reculé d'un rayon de
   socle -> centre du socle.
3. Inclinaison (grossière) : hauteur de la boîte comparée à celle d'une pièce debout, avec la pose
   caméra estimée par solvePnP (focale supposée). Boîte plus large que haute -> pièce couchée.

Repère plateau : origine au coin extérieur de a1, x vers la colonne h, y vers la rangée 8.
Le plateau est symétrique : a1 = coin foncé le plus bas dans l'image (flip=True pour 180°).

Test hors ROS sur une photo, avec les détections YOLO déjà sauvées :
    python3 -m so101_perception.chess_vision images/image2.jpg --labels runs/detect/predict-10/labels/image2.txt
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

FILES = "abcdefgh"
WORK_WIDTH = 1600  # largeur de l'image de travail pour la détection du plateau
SQUARE_SIZE = 0.025  # côté d'une case (m), à mesurer

# Hauteur et rayon du socle (m), mesurés sur les photos. Transmis dans l'état publié.
PIECE_SPECS = {
    "pawn":   dict(height=0.040, base_r=0.0105),
    "rook":   dict(height=0.052, base_r=0.0115),
    "knight": dict(height=0.055, base_r=0.0115),
    "bishop": dict(height=0.064, base_r=0.0110),
    "queen":  dict(height=0.068, base_r=0.0115),
    "king":   dict(height=0.078, base_r=0.0125),
}

MIN_TILT_DEG = 30.0      # en dessous : considérée droite (mesure trop imprécise)
FALLEN_TILT_DEG = 45.0
MAX_ELEVATION_DEG = 65.0  # caméra plus haute : pas d'estimation d'inclinaison
NMS_IOU = 0.5


# --------------------------------------------------------------------------- plateau

def square_of(xy_board, square_size: float = SQUARE_SIZE) -> str | None:
    f = int(np.floor(xy_board[0] / square_size))
    r = int(np.floor(xy_board[1] / square_size))
    return f"{FILES[f]}{r + 1}" if 0 <= f < 8 and 0 <= r < 8 else None



@dataclass
class Board:
    H_img2board: np.ndarray  # homographie pixel -> plateau (mètres)
    square_size: float
    corners_px: np.ndarray  # coins extérieurs en pixels : a1, h1, h8, a8
    n_squares_used: int

    @property
    def H_board2img(self) -> np.ndarray:
        return np.linalg.inv(self.H_img2board)

    def img_to_board(self, pts_px) -> np.ndarray:
        return _project(self.H_img2board, pts_px)

    def board_to_img(self, pts_board) -> np.ndarray:
        return _project(self.H_board2img, pts_board)

    def square_of(self, xy) -> str | None:
        """Nom de la case (ex. 'e4') contenant le point plateau xy, None si hors plateau."""
        return square_of(xy, self.square_size)

    def flipped(self) -> "Board":
        """Même plateau tourné de 180° (a1 <-> h8)."""
        c = np.roll(self.corners_px, 2, axis=0)
        return board_from_corners(c, self.square_size, self.n_squares_used)


def board_from_corners(corners_px, square_size: float, n_used: int = 0) -> Board:
    """corners_px : coins extérieurs a1, h1, h8, a8 (pixels de l'image d'origine)."""
    corners_px = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    L = 8 * square_size
    dst = np.array([(0, 0), (L, 0), (L, L), (0, L)], dtype=np.float64)
    H = cv2.getPerspectiveTransform(np.float32(corners_px), np.float32(dst))
    return Board(H_img2board=H, square_size=square_size, corners_px=corners_px, n_squares_used=n_used)


# --------------------------------------------------------------------------- outils

def _project(H, pts):
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H).reshape(-1, 2)


def _dark_quads(gray):
    """Centres et coins des quadrilatères sombres (candidats cases noires)."""
    quads = []
    for block in (31, 51, 81):
        th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, block, 10)
        for it in (1, 2, 3):
            er = cv2.erode(th, np.ones((3, 3), np.uint8), iterations=it)
            cnts, _ = cv2.findContours(er, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                a = cv2.contourArea(c)
                if a < 150:
                    continue
                hull = cv2.convexHull(c)
                ap = cv2.approxPolyDP(hull, 0.08 * cv2.arcLength(hull, True), True)
                if len(ap) != 4 or not cv2.isContourConvex(ap):
                    continue
                if a / max(cv2.contourArea(ap), 1) < 0.8:
                    continue
                p = ap.reshape(4, 2).astype(np.float64)
                sides = np.linalg.norm(p - np.roll(p, 1, 0), axis=1)
                if sides.max() / sides.min() > 3:
                    continue
                if _is_dark(gray, p):
                    quads.append(p)
    # dédoublonnage : un seul quad (le plus grand) par centre
    quads.sort(key=lambda p: -cv2.contourArea(np.float32(p)))
    kept, centers = [], []
    for p in quads:
        c = p.mean(0)
        size = np.sqrt(cv2.contourArea(np.float32(p)))
        if all(np.linalg.norm(c - k) > 0.4 * size for k in centers):
            kept.append(p)
            centers.append(c)
    return kept, np.array(centers)


def _is_dark(gray, quad):
    """Le quad est plus sombre que son voisinage (élimine les cases blanches vues comme des
    trous dans la zone noire)."""
    c = quad.mean(0)
    inner = np.zeros(gray.shape, np.uint8)
    cv2.fillPoly(inner, [np.int32(c + 0.7 * (quad - c))], 255)
    outer = np.zeros(gray.shape, np.uint8)
    cv2.fillPoly(outer, [np.int32(c + 1.8 * (quad - c))], 255)
    cv2.fillPoly(outer, [np.int32(c + 1.1 * (quad - c))], 0)
    if not inner.any() or not outer.any():
        return False
    return gray[inner > 0].mean() < 0.75 * gray[outer > 0].mean()


def _fit_lattice(quads, centers):
    """RANSAC : renvoie (H grille->image, coords grille des cases noires inliers)."""
    unit = np.array([(0, 0), (1, 0), (1, 1), (0, 1)], dtype=np.float64)
    best = None
    for p in quads:
        H = cv2.getPerspectiveTransform(np.float32(unit), np.float32(p))  # case graine = [0,1]²
        lat = None
        for _ in range(6):
            if abs(np.linalg.det(H)) < 1e-12:
                break
            uv = _project(np.linalg.inv(H), centers) - 0.5
            ij = np.round(uv)
            res = np.linalg.norm(uv - ij, axis=1)
            ok = (res < 0.22) & (((ij[:, 0] + ij[:, 1]) % 2) == 0) & (np.abs(ij).max(1) <= 8)
            # une seule case détectée par cellule de la grille (la plus proche)
            cells = {}
            for k in np.flatnonzero(ok):
                key = (int(ij[k, 0]), int(ij[k, 1]))
                if key not in cells or res[k] < res[cells[key]]:
                    cells[key] = k
            if len(cells) < 4:
                break
            idx = np.array(list(cells.values()))
            lat = ij[idx]
            Hn, _ = cv2.findHomography(lat + 0.5, centers[idx], cv2.RANSAC, 4.0)
            if Hn is None:
                break
            H = Hn
        if lat is None:
            continue
        span = lat.max(0) - lat.min(0)
        if span.max() > 7:  # plus grand que le plateau : faux positif
            continue
        if best is None or len(lat) > len(best[1]):
            best = (H, lat)
    return best


def _place_on_board(gray, H, lat):
    """Décalage (oi, oj) tel que case_grille + (oi, oj) = case du plateau (0..7).
    Choisi en comparant les couleurs attendues des cases (damier dedans, blanc autour)."""
    h, w = gray.shape
    imin, jmin = lat.min(0).astype(int)
    imax, jmax = lat.max(0).astype(int)
    best = None
    for oi in range(-imin, 8 - imax):
        for oj in range(-jmin, 8 - jmax):
            parity = (oi + oj) % 2  # cases noires du plateau : (bi + bj) % 2 == parity
            score = 0
            for bi in range(-1, 9):
                for bj in range(-1, 9):
                    c = _project(H, [(bi - oi + 0.5, bj - oj + 0.5)])[0]
                    x, y = int(c[0]), int(c[1])
                    if not (5 <= x < w - 5 and 5 <= y < h - 5):
                        continue
                    v = float(np.median(gray[y - 4 : y + 5, x - 4 : x + 5]))
                    inside = 0 <= bi < 8 and 0 <= bj < 8
                    expected_dark = inside and (bi + bj) % 2 == parity
                    score += 1 if (v < 100) == expected_dark else -1
            if best is None or score > best[0]:
                best = (score, oi, oj)
    return best


def _refine_with_corners(gray, H, rounds=2):
    """Affine l'homographie avec les coins de cases visibles (plus précis que les centres des
    cases noires) : chaque coin prédit est ajusté par cornerSubPix et gardé s'il ressemble à
    un coin de damier (deux quadrants opposés sombres, les deux autres clairs)."""
    h, w = gray.shape
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    for _ in range(rounds):
        lat, px = [], []
        for i in range(-8, 9):
            for j in range(-8, 9):
                p = _project(H, [(i, j)])[0]
                step = np.linalg.norm(_project(H, [(i + 1, j)])[0] - p)
                win = int(max(3, 0.3 * step))
                if not (win + 2 < p[0] < w - win - 2 and win + 2 < p[1] < h - win - 2):
                    continue
                q = _project(H, [(i - 0.3, j - 0.3), (i + 0.3, j - 0.3), (i + 0.3, j + 0.3), (i - 0.3, j + 0.3)])
                if not ((q >= 2) & (q < [w - 2, h - 2])).all():
                    continue
                v = [float(blur[int(y), int(x)]) for x, y in q]
                if abs((v[0] + v[2]) - (v[1] + v[3])) / 2 < 40 or abs(v[0] - v[2]) > 40 or abs(v[1] - v[3]) > 40:
                    continue  # pas un coin de damier (caché par une pièce, ou bord)
                r = cv2.cornerSubPix(gray, np.float32([[p]]), (win, win), (-1, -1),
                                     (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)).reshape(2)
                if np.linalg.norm(r - p) < 0.25 * step:
                    lat.append((i, j))
                    px.append(r)
        if len(lat) < 8:
            return H
        Hn, _ = cv2.findHomography(np.float64(lat), np.float64(px), cv2.RANSAC, 2.0)
        if Hn is None:
            return H
        H = Hn
    return H


def detect_board(img_bgr: np.ndarray, square_size: float = SQUARE_SIZE, hide=None) -> Board:
    """hide : coins (4, 2) d'un marqueur ArUco à masquer (ses carrés noirs ressemblent à des cases)."""
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    if hide is not None:
        c = hide.mean(0)
        gray = gray.copy()
        cv2.fillConvexPoly(gray, np.int32(c + 1.3 * (hide - c)), 255)
    scale = WORK_WIDTH / gray.shape[1]
    small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    quads, centers = _dark_quads(small)
    if len(quads) < 4:
        raise RuntimeError("Pas assez de cases noires visibles. Utiliser --corners.")
    fit = _fit_lattice(quads, centers)
    if fit is None:
        raise RuntimeError("Aucune grille de damier trouvée. Utiliser --corners.")
    H, lat = fit
    # seuil noir/blanc adaptatif pour le placement
    norm = cv2.normalize(small, None, 0, 255, cv2.NORM_MINMAX)
    placed = _place_on_board(norm, H, lat)
    if placed is None:
        raise RuntimeError("Impossible de situer la grille dans le plateau 8x8. Utiliser --corners.")
    _, oi, oj = placed
    H = _refine_with_corners(small, H)

    # coins extérieurs (coords plateau en cases) -> coords grille -> pixels
    outer = np.array([(0, 0), (8, 0), (8, 8), (0, 8)], dtype=np.float64) - (oi, oj)
    outer_px = _project(H, outer) / scale
    # Coins foncés du plateau : (0,0)/(8,8) si parité paire, sinon (8,0)/(0,8).
    # a1 (case foncée) = le coin foncé le plus bas dans l'image.
    dark = (0, 2) if (oi + oj) % 2 == 0 else (1, 3)
    a1 = max(dark, key=lambda k: outer_px[k][1])
    corners = np.array([outer_px[(a1 + k) % 4] for k in range(4)])  # a1, h1, h8, a8 ou a1, a8, h8, h1
    # Repère direct vu du dessus : avec y image vers le bas, a1->h1 puis a1->a8 tourne en sens horaire
    v1, v2 = corners[1] - corners[0], corners[3] - corners[0]
    if v1[0] * v2[1] - v1[1] * v2[0] > 0:
        corners = corners[[0, 3, 2, 1]]
    return board_from_corners(corners, square_size, n_used=len(lat))


def draw_board(img, board: Board):
    out = img.copy()
    s = board.square_size
    th = max(2, img.shape[1] // 800)
    for k in range(9):
        for a, b in (((k * s, 0), (k * s, 8 * s)), ((0, k * s), (8 * s, k * s))):
            p = board.board_to_img([a, b]).astype(int)
            cv2.line(out, tuple(map(int, p[0])), tuple(map(int, p[1])), (0, 255, 0), th)
    for name, (x, y) in (("a1", (0.5, 0.5)), ("h1", (7.5, 0.5)), ("a8", (0.5, 7.5))):
        p = board.board_to_img([(x * s, y * s)])[0].astype(int)
        cv2.putText(out, name, (int(p[0]) - 20, int(p[1]) + 10), cv2.FONT_HERSHEY_SIMPLEX, th * 0.6, (0, 0, 255), th)
    return out

# --------------------------------------------------------------------------- position du bras (ArUco)
# Un marqueur ArUco scotché sur la table, à un endroit fixe et mesuré une fois par rapport au
# bras, donne la position du plateau par rapport au bras à chaque image : le plateau et le
# marqueur sont sur le même plan (la table), l'homographie du plateau donne donc la position du
# marqueur dans le repère plateau, et on en déduit le passage plateau -> bras.

ARUCO_DICT = cv2.aruco.DICT_4X4_50


def find_marker(img, marker_id=0):
    """Coins (4, 2) du marqueur dans l'image (haut-gauche, haut-droit, bas-droit, bas-gauche du
    marqueur imprimé), ou None."""
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), cv2.aruco.DetectorParameters())
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None:
        return None
    for c, i in zip(corners, ids.ravel()):
        if int(i) == marker_id:
            return c.reshape(4, 2).astype(np.float64)
    return None


def _rot(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def marker_corners_robot(size, x, y, yaw_deg):
    """Coins du marqueur dans le repère du bras. (x, y) : centre du marqueur ; yaw : direction
    du haut du marqueur imprimé (0 = vers l'avant du bras, x)."""
    h = size / 2
    # repère marqueur : u vers la droite de l'image imprimée, v vers le haut
    local = np.array([(-h, h), (h, h), (h, -h), (-h, -h)])
    # haut du marqueur (v) orienté à yaw ; donc u (droite) à yaw - 90°
    R = _rot(yaw_deg - 90.0)
    return np.array([x, y]) + local @ R.T


def board_pose_from_marker(board: "Board", marker_px, size, x, y, yaw_deg):
    """Pose du plateau dans le repère du bras : (x, y) du coin a1 (m), yaw (°) de l'axe a->h."""
    p_board = board.img_to_board(marker_px)
    p_robot = marker_corners_robot(size, x, y, yaw_deg)
    # transformation rigide 2D robot = t + R @ board (Kabsch)
    cb, cr = p_board.mean(0), p_robot.mean(0)
    H = (p_board - cb).T @ (p_robot - cr)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:  # pas de symétrie miroir
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = cr - R @ cb
    err = float(np.abs(p_board @ R.T + t - p_robot).max())
    return dict(x=round(float(t[0]), 4), y=round(float(t[1]), 4),
                yaw=round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 2), err=round(err, 4))


def make_marker_sheet(path, size=0.08, marker_id=0, dpi=300):
    """Feuille A4 à imprimer à 100 % : marqueur de `size` m, flèche vers le haut du marqueur."""
    px = lambda m: int(round(m / 0.0254 * dpi))
    page = np.full((px(0.297), px(0.210)), 255, np.uint8)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), marker_id, px(size))
    y0, x0 = (page.shape[0] - marker.shape[0]) // 2, (page.shape[1] - marker.shape[1]) // 2
    page[y0:y0 + marker.shape[0], x0:x0 + marker.shape[1]] = marker
    cx = page.shape[1] // 2
    cv2.arrowedLine(page, (cx, y0 - px(0.01)), (cx, y0 - px(0.04)), 0, 6, tipLength=0.3)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(page, "HAUT : vers l'avant du bras (MARKER_YAW = 0)", (px(0.02), y0 - px(0.05)), font, 1.6, 0, 3)
    cv2.putText(page, f"ArUco 4x4 id {marker_id} - cote {size * 100:.1f} cm (verifier a la regle)",
                (px(0.02), y0 + marker.shape[0] + px(0.02)), font, 1.6, 0, 3)
    cv2.putText(page, "Imprimer a 100 % (sans mise a l'echelle)", (px(0.02), y0 + marker.shape[0] + px(0.035)), font, 1.6, 0, 3)
    if str(path).lower().endswith(".pdf"):  # page A4 exacte : s'imprime sans mise à l'échelle
        from PIL import Image

        Image.fromarray(page).save(str(path), resolution=dpi)
    else:
        cv2.imwrite(str(path), page)
    return path


# --------------------------------------------------------------------------- pièces

def _overlap(a, b):
    """(IoU, part de la plus petite boîte recouverte)."""
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area = lambda r: (r[2] - r[0]) * (r[3] - r[1])
    return inter / (area(a) + area(b) - inter + 1e-9), inter / (min(area(a), area(b)) + 1e-9)


def nms_class_agnostic(dets, iou=NMS_IOU, contained=0.8):
    """Le modèle sort parfois 2 classes pour la même pièce (reine/roi), ou une petite boîte sur
    une partie d'une pièce (tête du cavalier vue comme un pion) : on garde la plus sûre, et à
    confiance égale la plus grande."""
    area = lambda r: (r[2] - r[0]) * (r[3] - r[1])
    dets = sorted(dets, key=lambda d: (-d["conf"], -area(d["xyxy"])))
    kept = []
    for d in dets:
        if all(
            (lambda o: o[0] < iou and o[1] < contained)(_overlap(d["xyxy"], k["xyxy"])) for k in kept
        ):
            kept.append(d)
    return kept


# --------------------------------------------------------------------------- géométrie

def camera_pose(board: Board, img_shape, fx_ratio: float):
    """K supposée (focale = fx_ratio * largeur, centre optique au centre) + pose par PnP."""
    h, w = img_shape[:2]
    f = fx_ratio * w
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], dtype=np.float64)
    L = 8 * board.square_size
    obj = np.array([(0, 0, 0), (L, 0, 0), (L, L, 0), (0, L, 0)], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, board.corners_px, K, None, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise RuntimeError("solvePnP a échoué")
    return K, rvec, tvec


def base_center(board: Board, bbox, base_r):
    """Centre du socle (repère plateau) à partir du bas de la boîte."""
    x1, y1, x2, y2 = bbox
    u, v = (x1 + x2) / 2, y2
    p = board.img_to_board([(u, v)])[0]
    # direction « s'éloigner de la caméra » sur le plateau = remonter dans l'image
    q = board.img_to_board([(u, v - 0.05 * (y2 - y1))])[0]
    d = q - p
    n = np.linalg.norm(d)
    return p if n < 1e-9 else p + base_r * d / n


def camera_elevation_deg(rvec, tvec):
    """Angle de la caméra au-dessus du plateau (90° = vue de dessus)."""
    R, _ = cv2.Rodrigues(rvec)
    cam = (-R.T @ tvec).ravel()  # position caméra dans le repère plateau
    centre = np.array([4, 4, 0]) * SQUARE_SIZE
    d = cam - centre
    return float(np.degrees(np.arctan2(d[2], np.linalg.norm(d[:2]))))


def estimate_tilt(bbox, center_xy, height, K, rvec, tvec):
    """Inclinaison (deg) déduite du rapport hauteur observée / hauteur attendue debout."""
    x1, y1, x2, y2 = bbox
    if (x2 - x1) > 1.2 * (y2 - y1):
        return 90.0
    if camera_elevation_deg(rvec, tvec) > MAX_ELEVATION_DEG:
        return 0.0  # vue de dessus : la hauteur apparente ne dit rien sur l'inclinaison
    pts = np.array([[center_xy[0], center_xy[1], 0.0], [center_xy[0], center_xy[1], height]])
    img, _ = cv2.projectPoints(pts, rvec, tvec, K, None)
    img = img.reshape(-1, 2)
    expected = np.linalg.norm(img[1] - img[0])
    observed = y2 - y1
    ratio = np.clip(observed / max(expected, 1e-6), 0.0, 1.0)
    tilt = float(np.degrees(np.arccos(ratio)))
    return tilt if tilt >= MIN_TILT_DEG else 0.0


def draw_pieces(img, board, pieces, rejected):
    out = draw_board(img, board)
    th = max(2, img.shape[1] // 800)
    for p in pieces:
        x1, y1, x2, y2 = map(int, p["bbox"])
        color = (0, 0, 255) if p["fallen"] else (255, 128, 0)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, th)
        c = board.board_to_img([(p["x"], p["y"])])[0].astype(int)
        cv2.circle(out, (int(c[0]), int(c[1])), 4 * th, (0, 0, 255), -1)
        label = f"{p['type']} {p['square']}" + (f" {p['tilt_deg']:.0f}deg" if p["tilt_deg"] else "")
        cv2.putText(out, label, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * th, color, th)
    for d in rejected:
        x1, y1, x2, y2 = map(int, d["xyxy"])
        cv2.rectangle(out, (x1, y1), (x2, y2), (128, 128, 128), 1)
    return out


def analyse(img, dets, board: Board, fx_ratio=0.72):
    """Détections -> liste de pièces (dict) + détections hors plateau."""
    try:
        cam = camera_pose(board, img.shape, fx_ratio)
    except RuntimeError:
        cam = None  # pas d'inclinaison sans pose caméra
    pieces, rejected, counts = [], [], {}
    for d in nms_class_agnostic(dets):
        color, ptype = d["name"].split()
        spec = PIECE_SPECS[ptype]
        xy = base_center(board, d["xyxy"], spec["base_r"])
        square = board.square_of(xy)
        if square is None:
            rejected.append(d)
            continue
        tilt = estimate_tilt(d["xyxy"], xy, spec["height"], *cam) if cam else 0.0
        key = f"{color}_{ptype}"
        counts[key] = counts.get(key, 0) + 1
        pieces.append(dict(
            id=f"{key}_{counts[key]}", color=color, type=ptype, square=square,
            x=round(float(xy[0]), 4), y=round(float(xy[1]), 4),
            tilt_deg=round(tilt, 1), fallen=tilt >= FALLEN_TILT_DEG,
            height=spec["height"], base_r=spec["base_r"],
            conf=round(d["conf"], 3), bbox=[round(v, 1) for v in d["xyxy"]],
        ))
    # une seule pièce par case : la plus sûre
    best = {}
    for p in pieces:
        if p["square"] not in best or p["conf"] > best[p["square"]]["conf"]:
            best[p["square"]] = p
    return list(best.values()), rejected


def vote(history, min_ratio=0.6):
    """Fusionne plusieurs images : garde une case si elle est occupée dans au moins min_ratio
    des images, avec le type/couleur le plus fréquent et la position médiane."""
    n = len(history)
    by_square = {}
    for pieces in history:
        for p in pieces:
            by_square.setdefault(p["square"], []).append(p)
    out, counts = [], {}
    for square, ps in sorted(by_square.items()):
        if len(ps) < min_ratio * n:
            continue
        labels = [(p["color"], p["type"]) for p in ps]
        color, ptype = max(set(labels), key=labels.count)
        same = [p for p in ps if (p["color"], p["type"]) == (color, ptype)]
        key = f"{color}_{ptype}"
        counts[key] = counts.get(key, 0) + 1
        tilt = float(np.median([p["tilt_deg"] for p in same]))
        out.append(dict(
            same[0], id=f"{key}_{counts[key]}",
            x=round(float(np.median([p["x"] for p in same])), 4),
            y=round(float(np.median([p["y"] for p in same])), 4),
            tilt_deg=round(tilt, 1), fallen=tilt >= FALLEN_TILT_DEG,
            conf=round(len(same) / n, 2),  # ici : part des images où la pièce est vue
        ))
    return out


def detections_from_yolo(model, img, conf=0.25):
    r = model.predict(img, conf=conf, verbose=False)[0]
    return [
        dict(name=r.names[int(c)], conf=float(p), xyxy=[float(v) for v in xyxy])
        for c, p, xyxy in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(), r.boxes.xyxy.tolist())
    ]


def detections_from_labels(label_path, img_shape, names):
    """Fichier YOLO txt (cls cx cy w h [conf]) -> détections, pour tester sans le modèle."""
    h, w = img_shape[:2]
    out = []
    for line in Path(label_path).read_text().splitlines():
        p = line.split()
        if len(p) < 5:
            continue
        cls, cx, cy, bw, bh = int(p[0]), *map(float, p[1:5])
        xyxy = [(cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h]
        out.append(dict(name=names[cls], conf=float(p[5]) if len(p) > 5 else 1.0, xyxy=xyxy))
    return out


def read_state(img, dets, square_size=SQUARE_SIZE, fx_ratio=0.72, flip=False, board=None):
    """Photo + détections -> (état du plateau, image annotée, plateau).
    board : plateau déjà trouvé (sinon détecté ici ; RuntimeError s'il est introuvable)."""
    if board is None:
        board = detect_board(img, square_size)
        if flip:
            board = board.flipped()
    pieces, rejected = analyse(img, dets, board, fx_ratio)
    state = {"square_size": square_size, "pieces": pieces}
    return state, draw_pieces(img, board, pieces, rejected), board


def main():
    import yaml

    ap = argparse.ArgumentParser()
    ap.add_argument("image", nargs="?")
    ap.add_argument("--marker-sheet", help="écrit la feuille A4 du marqueur ArUco (aruco.pdf) et s'arrête")
    ap.add_argument("--labels", help="détections YOLO sauvées (txt) au lieu de lancer le modèle")
    ap.add_argument("--weights", default="runs/detect/train-17/weights/best.pt")
    ap.add_argument("--data", default="dataset/data.yaml", help="noms des classes pour --labels")
    ap.add_argument("--flip", action="store_true")
    ap.add_argument("--out", default="runs/scene")
    args = ap.parse_args()

    if args.marker_sheet:
        print(f"-> {make_marker_sheet(args.marker_sheet)} (imprimer à 100 %)")
        return
    img = cv2.imread(args.image)
    if args.labels:
        names = {int(k): v for k, v in yaml.safe_load(Path(args.data).read_text())["names"].items()}
        dets = detections_from_labels(args.labels, img.shape, names)
    else:
        from ultralytics import YOLO

        dets = detections_from_yolo(YOLO(args.weights), img)
    state, annotated, _ = read_state(img, dets, flip=args.flip)
    for p in sorted(state["pieces"], key=lambda p: p["square"]):
        print(f"  {p['square']}  {p['color']:<5} {p['type']:<6} ({p['x'] * 100:5.1f}, {p['y'] * 100:5.1f}) cm  "
              f"incl {p['tilt_deg']:.0f}°  conf {p['conf']:.2f}")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stem = Path(args.image).stem
    (out / f"{stem}.json").write_text(json.dumps(state, indent=1))
    cv2.imwrite(str(out / f"{stem}.jpg"), annotated)
    print(f"-> {out / stem}.json")


if __name__ == "__main__":
    main()
