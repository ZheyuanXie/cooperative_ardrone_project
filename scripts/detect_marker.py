import cv2
import numpy as np

GENERATOR_MATRIX = np.matrix([
    [1, 1, 0, 1],
    [1, 0, 1, 1],
    [1, 0, 0, 0],
    [0, 1, 1, 1],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])

REGENERATOR_MATRIX = np.matrix([
    [0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 1],
])

PARITY_CHECK_MATRIX = np.matrix([
    [1, 0, 1, 0, 1, 0, 1],
    [0, 1, 1, 0, 0, 1, 1],
    [0, 0, 0, 1, 1, 1, 1],
])

HAMMINGCODE_MARKER_POSITIONS = [
    [1, 2], [1, 3], [1, 4],
    [2, 1], [2, 2], [2, 3], [2, 4], [2, 5],
    [3, 1], [3, 2], [3, 3], [3, 4], [3, 5],
    [4, 1], [4, 2], [4, 3], [4, 4], [4, 5],
    [5, 2], [5, 3], [5, 4],
]


BORDER_COORDINATES = [
    [0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [1, 0], [1, 6], [2, 0], [2, 6], [3, 0],
    [3, 6], [4, 0], [4, 6], [5, 0], [5, 6], [6, 0], [6, 1], [6, 2], [6, 3], [6, 4], [6, 5], [6, 6],
]

ORIENTATION_MARKER_COORDINATES = [[1, 1], [1, 5], [5, 1], [5, 5]]

def extract_hamming_code(mat):
    hamming_code = ''
    for pos in HAMMINGCODE_MARKER_POSITIONS:
        hamming_code += str(int(mat[pos[0], pos[1]]))
    return hamming_code

def decode(bits):
    decoded_code = ''
    if len(bits) % 7 != 0:
        raise ValueError('Only a multiple of 7 as bits are allowed.')
    for bit in bits:
        if int(bit) not in [0, 1]:
            raise ValueError('The provided bits contain other values that 0 or 1: %s' % bits)
    while len(bits) >= 7:
        seven_bits = bits[:7]
        uncorrected_bit_array = generate_bit_array(seven_bits)
        corrected_bit_array = parity_correct(uncorrected_bit_array)
        decoded_bits = matrix_array_multiply_and_format(REGENERATOR_MATRIX, corrected_bit_array)
        decoded_code += ''.join(decoded_bits)
        bits = bits[7:]
    return decoded_code



def validate_and_turn(marker):
	    # first, lets make sure that the border contains only zeros
	    for crd in BORDER_COORDINATES:
	        if marker[crd[0], crd[1]] != 0.0:
	            raise ValueError('Border contians not entirely black parts.')
	    # search for the corner marker for orientation and make sure, there is only 1
	    orientation_marker = None
	    for crd in ORIENTATION_MARKER_COORDINATES:
	        marker_found = False
	        if marker[crd[0], crd[1]] == 1.0:
	            marker_found = True
	        if marker_found and orientation_marker:
	            raise ValueError('More than 1 orientation_marker found.')
	        elif marker_found:
	            orientation_marker = crd
	    if not orientation_marker:
	        raise ValueError('No orientation marker found.')
	    rotation = 0
	    if orientation_marker == [1, 5]:
	        rotation = 1
	    elif orientation_marker == [5, 5]:
	        rotation = 2
	    elif orientation_marker == [5, 1]:
	        rotation = 3
	    marker = rot90(marker, k=rotation)
	    return marker

def detect_markers(img):
    width, height, _ = img.shape
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray, 10, 100)
    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:]

    # We only keep the long enough contours
    min_contour_length = min(width, height) / 50
    contours = [contour for contour in contours if len(contour) > min_contour_length]
    warped_size = 49
    canonical_marker_coords = np.array(((0, 0),
                                     (warped_size - 1, 0),
                                     (warped_size - 1, warped_size - 1),
                                     (0, warped_size - 1)),
                                    dtype='float32')

    markers_list = []
    for contour in contours:
        approx_curve = cv2.approxPolyDP(contour, len(contour) * 0.01, True)
        if not (len(approx_curve) == 4 and cv2.isContourConvex(approx_curve)):
            continue

        sorted_curve = np.array(cv2.convexHull(approx_curve, clockwise=False),
                             dtype='float32')
        persp_transf = cv2.getPerspectiveTransform(sorted_curve, canonical_marker_coords)
        warped_img = cv2.warpPerspective(img, persp_transf, (warped_size, warped_size))
        warped_gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)

        _, warped_bin = cv2.threshold(warped_gray, 127, 255, cv2.THRESH_BINARY)
        marker = warped_bin.reshape(
            [7, warped_size / 7, 7, warped_size / 7]
        )
        marker = marker.mean(axis=3).mean(axis=1)
        marker[marker < 127] = 0
        marker[marker >= 127] = 1

        print marker

        try:
            marker = validate_and_turn(marker)
            hamming_code = extract_hamming_code(marker)
            marker_id = int(decode(hamming_code), 2)
            markers_list.append(HammingMarker(id=marker_id, contours=approx_curve))
        except ValueError:
            continue
    return markers_list