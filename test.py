import cv2


def process_frame(img):
    print(img.shape)


if __name__ == '__main__':
    cap = cv2.VideoCapture('./test.mp4')

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            process_frame(frame)
        else:
            break
