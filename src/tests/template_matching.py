import numpy as np
import cv2
def temp_match(img, templ):
    template = [templ]

    frame = img
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    x_loc = []
    y_loc = []
    y = []
    height = []
    for i in range(5):
        gray_template = cv2.cvtColor(template[0], cv2.COLOR_BGR2GRAY)
        resize_i = cv2.resize(gray, None,fx=1/(2**(0.5*i)), \
                             fy=1/(2**(0.5*i)),interpolation = cv2.INTER_AREA)
        res = cv2.matchTemplate(resize_i, gray_template, cv2.TM_CCOEFF_NORMED)
        loc = np.where(res >= 0.6)
        w,h = gray_template.shape[::-1]
        fac = int(2**(.5*i))

        for point in zip(*loc[::-1]):
            if len(x_loc) >= 1:
                thresh = 0
                for t in range (point[0]*fac - 20, point[0]*fac + 20):
                    if t in x_loc:
                        thresh = 1
                if thresh == 0:
                    cv2.rectangle(frame, (point[0]*fac, point[1]*fac), \
                    (point[0]*fac + w*fac, point[1]*fac + h*fac), (0,0,255), 2)
                    x_loc.append(point[0]*fac)
                    y_loc.append(point[1]*fac)
                    y.append((point[1] + h/2)*fac)
                    height.append(h*fac)
            else:
                cv2.rectangle(frame, (point[0]*fac, point[1]*fac), \
                (point[0]*fac + w*fac, point[1]*fac + h*fac), (0,0,255), 2)
                x_loc.append(point[0]*fac)
                y_loc.append(point[1]*fac)
                y.append((point[1] + h/2)*fac)
                height.append(h*fac)
            # print (x_loc,y_loc)
        if len(x_loc) >= 1:
            break
        else:
            x_loc = []
    cv2.imshow('Frame',frame)
    cv2.waitKey(10)
    # if len(x_loc) == 2 and len(y_loc) == 2:
    # y_loc = center of frame, x_loc = left bottom of all bounding poll boxes
    # height = height of poll in image
    return x_loc, np.mean(y), np.mean(height)
