import cv2 as cv
#from ultralytics.utils.benchmarks import benchmark

# Benchmark on GPU
#benchmark(model='/home/valvarn/Savasan2024/kenetlenme_gorevi/models/best_n.pt', 
#            data='/home/valvarn/Savasan2024/kenetlenme_gorevi/cfg/data.yaml', imgsz=640, half=False, device=0)

def draw_rectangle(event,x,y,flags,param):

    global pt1,pt2,topLeft_clicked,botRight_clicked

    # get mouse click
    if event == cv.EVENT_LBUTTONDOWN:

        if topLeft_clicked == True and botRight_clicked == True:
            topLeft_clicked = False
            botRight_clicked = False
            pt1 = (0,0)
            pt2 = (0,0)

        if topLeft_clicked == False:
            pt1 = (x,y)
            topLeft_clicked = True
            
        elif botRight_clicked == False:
            pt2 = (x,y)
            botRight_clicked = True

        
# Haven't drawn anything yet!

pt1 = (0,0)
pt2 = (0,0)
topLeft_clicked = False
botRight_clicked = False

cap = cv.VideoCapture(0) 

# Create a named window for connections
cv.namedWindow('Test')

# Bind draw_rectangle function to mouse cliks
cv.setMouseCallback('Test', draw_rectangle) 


while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if topLeft_clicked:
        cv.circle(frame, center=pt1, radius=5, color=(0,0,255), thickness=-1)
        
    #drawing rectangle
    if topLeft_clicked and botRight_clicked:
        cv.rectangle(frame, pt1, pt2, (0, 0, 255), 2)
        
    # Display the resulting frame
    cv.imshow('Test', frame)

    if cv.waitKey(1) & 0xFF == 27:
        break

# When everything is done, release the capture
cap.release()
cv.destroyAllWindows()