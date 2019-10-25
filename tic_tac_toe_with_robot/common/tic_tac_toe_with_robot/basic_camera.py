import numpy as np
import cv2
import datetime

def main():
    cap = cv2.VideoCapture(1)

    ans = raw_input("Record video? [y/N]: ")
    record = "y" in ans.lower()

    # Define the codec and create VideoWriter object
    if record:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        file_name = 'output_' + datetime.datetime.now().isoformat() + '.avi'
        out = cv2.VideoWriter(file_name, fourcc, 20.0, (640,480))

    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret==True:
            if record:
                flipped = cv2.flip(frame, 1)
                out.write(flipped)
                # out.write(frame)

            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    # Release everything if job is finished
    cap.release()
    if record:
        out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
