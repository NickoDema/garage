import matplotlib.pyplot as plt
from struct import *

def main():

    segments = []

    s0 = ""
    s1 = ""

    token = []



    with open('sokolov.txt','r') as f:
        for line in f:
            for word in line.split():
                s0 = s1
                s1 = word
                #print(word)
                if s0 == "aa" and s1 == "ff":
                    if len(token) > 10 :
                        segments.append(token)
                    token = []
                token.append(word)
                #if len(segments) > 1000:
                #    break


    #print(segments[1], segments[2])
    print(len(segments))

    steers1 = []

    min_steer_delta = 1000
    max_steer_delta = -1000
    prev_steer = 34.5

    step = 0
    wrong_step = 0
    steer_counter = 0

    for segment in segments:
    #for segment in segments:
        #print(segment)
        #dlc = int(segment[14])
        can_id = int(segment[9],16)
        can_line = int(segment[8],16)

        if can_id == 37 and can_line == 01:
            #print(segment[15],segment[16])
            steer = int(segment[15],16)*256 + int(segment[16],16)
            if steer > 0x800:
                steer -= 0x1000
            steer *= 1.5
            #print("Steer: {}".format(steer))
            steer_delta = steer - prev_steer
            if steer_delta > max_steer_delta:
                max_steer_delta = steer_delta
            if steer_delta < min_steer_delta:
                min_steer_delta = steer_delta
            #if steer != 34.5 and steer != -10.5:
            #if steer_counter >= 14000 and steer_counter < 14100:
            #if steer_counter >= 50 and steer_counter < 150:
            if steer_counter >= 0:
                steers1.append(steer)

                if abs(steer_delta) > 20:
                    print("step: {}; delta: {}".format(step, steer_delta))
                    wrong_step += 1

                #print(steer)
            prev_steer = steer

            step += 1
            steer_counter += 1

    steers2 = []

    for segment in segments:
    #for segment in segments:
        #print(segment)
        #dlc = int(segment[14])
        can_id = int(segment[9],16)
        can_line = int(segment[8],16)

        if can_id == 37 and can_line == 02:
            #print(segment[15],segment[16])
            steer = int(segment[15],16)*256 + int(segment[16],16)
            if steer > 0x800:
                steer -= 0x1000
            steer *= 1.5
            #print("Steer: {}".format(steer))
            steer_delta = steer - prev_steer
            if steer_delta > max_steer_delta:
                max_steer_delta = steer_delta
            if steer_delta < min_steer_delta:
                min_steer_delta = steer_delta
            #if steer != 34.5 and steer != -10.5:
            #if steer_counter >= 14000 and steer_counter < 14100:
            #if steer_counter >= 50 and steer_counter < 150:
            if steer_counter >= 0:
                steers2.append(steer)

                if abs(steer_delta) > 20:
                    print("step: {}; delta: {}".format(step, steer_delta))
                    wrong_step += 1

                #print(steer)
            prev_steer = steer

            step += 1
            steer_counter += 1

    print(min_steer_delta,max_steer_delta)
    print(step,wrong_step)

    plt.plot(steers1)
    plt.plot(steers2)
    plt.ylabel('Steering angle')
    plt.show()


if __name__ == "__main__":
    main()
