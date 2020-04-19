#!/usr/bin/env python

import cv2
import numpy as np
import time

def main():

    try:
        param_file = open('camera_param.txt', 'r')
    except:
        raise BlockingIOError('Can\'t find config file camera_param.txt ')
    params = []
    for line in param_file:
        for t in line.split():
            try:
                params.append(float(t))
            except ValueError:
                pass
    distance_to_object, object_width, matrix_width, matrix_height = params
    param_file.close()

    print('Distance to object: ' + str(distance_to_object))
    print('Cell width: ' + str(object_width))

    try:
        cap = cv2.VideoCapture("/dev/video0")
        cap.set(3, 1920)
        cap.set(4, 1080)
    except:
        raise Exception('Can\'t open video0')

    while True:
        rtv, frame = cap.read()

        full_frame_width = frame.shape[1]
        full_frame_height = frame.shape[0]

        frame = frame[frame.shape[0]//2-300:frame.shape[0]//2+300, frame.shape[1]//2-400:frame.shape[1]//2+400]

        frame_half_width = frame.shape[1]//2
        frame_half_height = frame.shape[0]//2

        canny = cv2.Canny(frame, 50, 150, apertureSize = 3)

        right_point = frame_half_width
        while True:
            got_edge = (canny[frame_half_height, right_point] + canny[frame_half_height-1, right_point] + canny[frame_half_height+1, right_point]) > 0
            end_of_frame = (right_point == frame.shape[1]-3)

            if got_edge or end_of_frame:
                break
            else:
                right_point+=1

        cv2.circle(canny, (right_point, frame_half_height), 7, 255, -1)
        cv2.circle(frame, (right_point, frame_half_height), 7, (200,100,100), -1)


        left_point = frame_half_width
        while True:
            got_edge = (canny[frame_half_height, left_point] + canny[frame_half_height-1, left_point] + canny[frame_half_height+1, left_point]) > 0
            end_of_frame = (left_point == 2)

            if got_edge or end_of_frame:
                break
            else:
                left_point-=1

        cv2.circle(canny, (left_point, frame_half_height), 7, 255, -1)
        cv2.circle(frame, (left_point, frame_half_height), 7, (200,100,100), -1)

        object_width_in_pixel = right_point - left_point
        if object_width_in_pixel != 0:
            d = (full_frame_width/float(object_width_in_pixel)*object_width)
            print(d)
            focus = matrix_width*distance_to_object/d
            cv2.putText(canny, '%4.2f' % focus, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, 255, thickness = 3)



        canny = cv2.resize(canny, (960, 540), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('canny', canny)

        frame = cv2.resize(frame, (960, 540), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('frame', frame)

        if cv2.waitKey(33) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # try:
    main()
    # except Exception as e:
    #     print(e)
