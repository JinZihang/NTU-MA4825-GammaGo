import math
import cv2
import numpy as np
import scipy.spatial as spatial
import scipy.cluster as cluster
from collections import defaultdict
from statistics import mean


def imgProcessing(frame, previous_cnrs, previous_intxns, history_bk):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)

    cnt = findContours(thresh)
    approx_cnrs = findApproxcnrs(cnt)
    is_board_area = True  # applyAreaConstraints(thresh)

    if len(approx_cnrs) == 4 and is_board_area:
        canvas = frame.copy()
        cv2.drawContours(canvas, cnt, -1, (0, 255, 0), 3)
        cv2.drawContours(canvas, approx_cnrs, -1, (255, 255, 0), 10)

        # Perspective transform.
        approx_cnrs = np.concatenate(approx_cnrs).tolist()
        H, W = thresh.shape
        ref_cnrs = [[0, H], [0, 0], [W, 0], [W, H]]
        sorted_cnrs = []
        for ref_cnr in ref_cnrs:
            x = [dist(ref_cnr, cnr) for cnr in approx_cnrs]
            min_posn = x.index(min(x))
            sorted_cnrs.append(approx_cnrs[min_posn])
        board_cnrs, h, w = getBoardCorners(sorted_cnrs)
        unwarpped = unwarpPerspective(
            frame, np.float32(sorted_cnrs), board_cnrs, w, h)

        # Remove the outer border of the board, else the board's border lines will be included in the line detection step.
        cropped_side_len = min(unwarpped.shape[0], unwarpped.shape[1])
        cropped = cv2.resize(unwarpped, (cropped_side_len, cropped_side_len))
        cropped = cropped[15:-15, 15:-15]

        # Update corners and intersections.
        if not np.array_equal(approx_cnrs, previous_cnrs):
            cropped_gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
            cropped_edges = getCannyEdges(cropped_gray)
            lines = cv2.HoughLines(cropped_edges, 1, np.pi / 180, 125, 100, 10)
            h_lines, v_lines = seperateHorVerLines(lines)

            if h_lines is not None and v_lines is not None:
                try:
                    intxn_pts = findIntersections(h_lines, v_lines)
                    intxn_clusters = clusterPoints(intxn_pts)
                    aug_intxns = augmentPoints(intxn_clusters)
                    if len(aug_intxns) != 100:
                        aug_intxns = previous_intxns.copy()
                        return cropped, previous_cnrs, previous_intxns, history_bk
                    return cropped, approx_cnrs, aug_intxns, history_bk
                except:
                    print("Failed to find line intersections.")
                    return cropped, previous_cnrs, previous_intxns, history_bk
        else:  # Find the black-stone positions.
            stone_frame = cropped.copy()
            bks = []
            for idx, intxn in enumerate(previous_intxns):
                if hasBlackStone(stone_frame, int(intxn[1]), int(intxn[0])):
                    bks.append(idx)
            return cropped, previous_cnrs, previous_intxns, bks
    return frame, previous_cnrs, previous_intxns, history_bk


def findContours(thresh):
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    return sorted(contours, key=cv2.contourArea, reverse=True)[0]


def findApproxcnrs(cnt):
    epsilon = 0.02 * cv2.arcLength(cnt, True)
    return cv2.approxPolyDP(cnt, epsilon, True)


def applyAreaConstraints(thresh):
    cnt_board_move = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt_board_move = cnt_board_move[0] if len(
        cnt_board_move) == 2 else cnt_board_move[1]
    for c in cnt_board_move:
        area = cv2.contourArea(c)
        if 10000 < area < 30000:
            return True
    return False


def dist(pt_1, pt_2):
    return math.sqrt((pt_1[0] - pt_2[0])**2 + (pt_1[1] - pt_2[1])**2)


def getBoardCorners(cnrs):
    # Corners: A -> B -> C -> D
    w1 = np.sqrt((cnrs[0][0] - cnrs[3][0]) ** 2 +
                 (cnrs[0][1] - cnrs[3][1]) ** 2)
    w2 = np.sqrt((cnrs[1][0] - cnrs[2][0]) ** 2 +
                 (cnrs[1][1] - cnrs[2][1]) ** 2)
    w = max(int(w1), int(w2))

    h1 = np.sqrt((cnrs[0][0] - cnrs[1][0]) ** 2 +
                 (cnrs[0][1] - cnrs[1][1]) ** 2)
    h2 = np.sqrt((cnrs[2][0] - cnrs[3][0]) ** 2 +
                 (cnrs[2][1] - cnrs[3][1]) ** 2)
    h = max(int(h1), int(h2))

    board_cnrs = np.float32([(0, h - 1), (0, 0), (w - 1, 0), (w - 1, h - 1)])
    return board_cnrs, h, w


def unwarpPerspective(img, src, dst, w, h):
    H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
    return cv2.warpPerspective(img, H, (w, h), flags=cv2.INTER_LINEAR)


def getCannyEdges(img, sigma=0.33):
    v = np.median(img)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edges = cv2.Canny(img, lower, upper)
    return edges


def seperateHorVerLines(lines):
    h_lines, v_lines = [], []
    for line in lines:
        rho, theta = line[0]
        if theta < np.pi / 4 or theta > np.pi - np.pi / 4:
            v_lines.append([rho, theta])
        else:
            h_lines.append([rho, theta])
    return h_lines, v_lines


def findIntersections(h_lines, v_lines):
    pts = []
    for r_h, t_h in h_lines:
        for r_v, t_v in v_lines:
            a = np.array([[np.cos(t_h), np.sin(t_h)],
                         [np.cos(t_v), np.sin(t_v)]])
            b = np.array([r_h, r_v])
            inter_pt = np.linalg.solve(a, b)
            pts.append(inter_pt)
    return np.array(pts)


def clusterPoints(pts):  # Hierarchical cluster (by euclidean distance) intersection points
    dists = spatial.distance.pdist(pts)
    single_linkage = cluster.hierarchy.single(dists)
    flat_clusters = cluster.hierarchy.fcluster(single_linkage, 15, 'distance')
    cluster_dict = defaultdict(list)
    for i in range(len(flat_clusters)):
        cluster_dict[flat_clusters[i]].append(pts[i])
    cluster_values = cluster_dict.values()
    clusters = map(lambda arr: (np.mean(np.array(arr)[:, 0]), np.mean(
        np.array(arr)[:, 1])), cluster_values)
    return sorted(list(clusters), key=lambda k: [k[1], k[0]])


def augmentPoints(pts):
    pts_shape = list(np.shape(pts))
    augmented_pts = []
    for row in range(int(pts_shape[0] / 10)):
        start = row * 10
        end = (row * 10) + 9
        rw_pts = pts[start:end + 1]
        rw_y = []
        rw_x = []
        for pt in rw_pts:
            x, y = pt
            rw_y.append(y)
            rw_x.append(x)
        y_mean = mean(rw_y)
        for i in range(len(rw_x)):
            pt = (rw_x[i], y_mean)
            augmented_pts.append(pt)
    augmented_pts = sorted(augmented_pts, key=lambda k: [k[1], k[0]])
    return augmented_pts


def drawLine(img, line, color):
    rho, theta = line
    if not np.isnan(rho) and not np.isnan(theta):
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        cv2.line(img, (x1, y1), (x2, y2), color, 2)


def hasBlackStone(img, x, y):
    anlys_area = img[x-5:x+5, y-5:y+5]
    ave_clr_per_row = np.average(anlys_area, axis=0)
    ave_clr = np.average(ave_clr_per_row, axis=0)
    ave_clr = sum(ave_clr) / len(ave_clr)

    # bk_clr_con = np.load('./perception/black_stone_color_constraint.npy')
    bk_clr_con = [100]
    if ave_clr < bk_clr_con[0]:
        cv2.circle(img, (y, x), radius=5, color=(153, 255, 51), thickness=-1)
        return True
    else:
        return False


def areaConstraintsCalibration(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)

    cnt_board_move = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnt_board_move = cnt_board_move[0] if len(
        cnt_board_move) == 2 else cnt_board_move[1]
    for c in cnt_board_move:
        area = cv2.contourArea(c)
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.015 * peri, True)

        if len(approx) == 4:
            print("Current area is " + str(area) + ".")


def colorConstraintsCalibration(cropped, previous_intxns):
    for idx, intxn in enumerate(previous_intxns):
        bk_ave_clrs, mt_ave_clrs = ([],)*2

        analyse_area = cropped[int(intxn[1])-5: int(intxn[1])+5,
                               int(intxn[0])-5: int(intxn[0])+5]
        ave_clr_per_row = np.average(analyse_area, axis=0)
        ave_clr = np.average(ave_clr_per_row, axis=0)
        ave_clr = sum(ave_clr) / len(ave_clr)

        if (idx == 0 | idx == 9 | idx == 90 | idx == 99):
            bk_ave_clrs.append(ave_clr)
        else:
            mt_ave_clrs.append(ave_clr)

    np.save('black_stone_color_constraint.npy',
            np.asarray([(max(bk_ave_clrs) + min(mt_ave_clrs))/2]))


def noiseFiltering(history_bk, new_bks, temp_bk, fr_cnt):
    new_bk_idx = list(set(new_bks) - set(history_bk))
    if (len(new_bk_idx) > 0):
        if (temp_bk == []):
            return history_bk, new_bk_idx, 1
        else:
            if (temp_bk == new_bk_idx):
                if (fr_cnt > 15):
                    print("Index of the new black stone: ",
                          new_bk_idx[0], + ".")
                    history_bk.append(new_bk_idx[0])
                    return history_bk, [], 0
                else:
                    return history_bk, temp_bk, fr_cnt+1
            else:
                return history_bk, new_bk_idx, 0
