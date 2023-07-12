import cv2

name = "push_pull"
cap = cv2.VideoCapture("{}.mp4".format(name))
count = 0

while cap.isOpened():
    ret, img = cap.read()
    if ret:
        cv2.imshow("img",img)
    if count == 0:
        cv2.imwrite("{}_frame0.jpg".format(name),img)
    count += 1
    key = cv2.waitKey(1)
    if key == 27 or not ret: # Press esc to exist
        break

cv2.destroyAllWindows()
cap.release()
