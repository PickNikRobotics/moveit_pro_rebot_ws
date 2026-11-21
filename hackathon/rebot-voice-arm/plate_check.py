"""Did something land on the white plate? Compare the plate before and after a pick."""
import cv2, numpy as np

def plate_mask(img):
    """The largest bright, colorless blob: the paper plate, shrunk off its rim."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, 170), (180, 50, 255))
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(white)
    if n < 2:
        return None
    big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    m = (lab == big).astype(np.uint8) * 255
    cnt = max(cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
    hull = np.zeros_like(m); cv2.drawContours(hull, [cv2.convexHull(cnt)], -1, 255, -1)
    return cv2.erode(hull, np.ones((15, 15), np.uint8))

def changed_on_plate(before, after):
    """Fraction of the plate whose look changed, and the change mask."""
    mask = plate_mask(before)
    if mask is None:
        return 0.0, None
    a = cv2.GaussianBlur(before, (5, 5), 0).astype(int)
    b = cv2.GaussianBlur(after, (5, 5), 0).astype(int)
    diff = (np.abs(a - b).max(axis=2) > 45).astype(np.uint8) * 255
    diff = cv2.morphologyEx(diff, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)) & mask
    return float(diff.sum()) / float(mask.sum()), diff

def verdict(before_path, after_path, annotated_path=None, min_frac=0.01):
    before, after = cv2.imread(before_path), cv2.imread(after_path)
    frac, diff = changed_on_plate(before, after)
    ok = frac >= min_frac
    if annotated_path is not None and diff is not None:
        out = after.copy()
        out[diff > 0] = (0.5 * out[diff > 0] + (0, 127, 0)).astype(np.uint8)
        cv2.putText(out, f"{'ON PLATE' if ok else 'NOT ON PLATE'} ({frac:.1%})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 0) if ok else (0, 0, 255), 2)
        cv2.imwrite(annotated_path, out)
    return ok, frac
