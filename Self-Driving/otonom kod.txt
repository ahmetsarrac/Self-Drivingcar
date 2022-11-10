import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from imutils.perspective import four_point_transform
import RPi.GPIO as io
import math

camera=PiCamera()
camera.resolution=(320,240)
camera.framerate=32
rawCapture=PiRGBArray(camera,size=(320,240))
io.setmode(io.BCM)
PWM_MAX = 100
io.setwarnings(False)
time.sleep(3)


def main():
    
    io_TRIGGER = 20
    io_ECHO = 21
    io.setup(io_TRIGGER, io.OUT)
    io.setup(io_ECHO, io.IN)
    
    #motor başlangıç
    L_L_EN = 22 # leftmotor_in1_pin
    L_R_EN = 23 # leftmotor_in2_pin
    L_L_PWM = 18 # leftmotorpwm_pin_l
    L_R_PWM = 17 # leftmotorpwm_pin_r

    R_L_EN = 13 # rightmotor_in1_pin
    R_R_EN = 19 # rightmotor_in2_pin
    R_L_PWM = 5 # rightmotorpwm_pin_l
    R_R_PWM = 6 # rightmotorpwm_pin_r

    io.setup(L_L_EN, io.OUT)
    io.setup(L_R_EN, io.OUT)
    io.setup(R_L_EN, io.OUT)
    io.setup(R_R_EN, io.OUT)
    
    io.output(L_L_EN, True)
    io.output(L_R_EN, True)
    io.output(R_L_EN, True)
    io.output(R_R_EN, True)

    io.setup(L_L_PWM, io.OUT)
    io.setup(L_R_PWM, io.OUT)
    io.setup(R_L_PWM, io.OUT)
    io.setup(R_R_PWM, io.OUT)
    
    solmotorsol = io.PWM(L_L_PWM,100)
    solmotorsag = io.PWM(L_R_PWM,100)
    sagmotorsol = io.PWM(R_L_PWM,100)
    sagmotorsag = io.PWM(R_R_PWM,100)
    
    solmotorsag.start(0)
    solmotorsol.start(0)
    sagmotorsag.start(0)
    sagmotorsol.start(0)
    
    solmotorsol.ChangeDutyCycle(0)
    solmotorsag.ChangeDutyCycle(0)
    sagmotorsol.ChangeDutyCycle(0)
    sagmotorsag.ChangeDutyCycle(0)

    #Şerit eğim değer başlangıçları
    largestLeftLineSize = 0
    largestRightLineSize = 0
    largestLeftLine = (0,0,0,0)
    largestRightLine = (0,0,0,0)


    #Başlangıç değerler
    Kirmizi_renk=0
    Tabela_kontrol=0
    Tabela_okundu=0
    Tabela_okundu2=0
    Yesil_renk=0
    
    #trafik levhaları için
    lower_blue = np.array([90,200,50])
    upper_blue = np.array([110,255,255])
    
    #kırmızı renk değerlikleri
    alt_deger=np.array([0,0,140])
    ust_deger=np.array([60,60,255])

    #yeşil renk değerlikleri
    alt_deger_1=np.array([0,140,0])
    ust_deger_1=np.array([60,255,60])
 
    for frame in camera.capture_continuous(rawCapture,format="bgr",use_video_port=True):
        
        frame = frame.array 

        frameArea = frame.shape[0]*frame.shape[1]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        kernel = np.ones((3,3),np.uint8)
     
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
       
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        
        detectedTrafficSign = None
        
       
        largestArea = 0
        largestRect = None
        
        # kırmızı için
        filtre1=cv2.inRange(frame,alt_deger,ust_deger)
        filtre1=cv2.GaussianBlur(filtre1,(3,3),2)
        filtre1 = cv2.dilate(filtre1,np.ones((5,5),np.uint8))
        filtre1 = cv2.erode(filtre1,np.ones((5,5),np.uint8))
        intRows,intColums = filtre1.shape
        circles = cv2.HoughCircles(filtre1,cv2.HOUGH_GRADIENT,5,intRows/4)
        #yeşil için
        filtre2=cv2.inRange(frame,alt_deger_1,ust_deger_1)
        filtre2=cv2.GaussianBlur(filtre2,(3,3),2)
        filtre2 = cv2.dilate(filtre2,np.ones((5,5),np.uint8))
        filtre2 = cv2.erode(filtre2,np.ones((5,5),np.uint8))
        intRows,intColums = filtre2.shape
        circless = cv2.HoughCircles(filtre2,cv2.HOUGH_GRADIENT,5,intRows/4)


        io.output(io_TRIGGER, False)
        time.sleep(0.00001)
        io.output(io_TRIGGER, True)
        time.sleep(0.00001)
        io.output(io_TRIGGER, False)
        while io.input(io_ECHO) == 0:
            StartTime = time.time()
        while io.input(io_ECHO) == 1:
            StopTime = time.time()
        TimeElapsed = StopTime - StartTime
        distance = TimeElapsed * 17150
        distance = round(distance, 2)
        mesafe_serit=distance - 0.5
        

        if len(cnts) > 0:
            for cnt in cnts:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                sideOne = np.linalg.norm(box[0]-box[1])
                sideTwo = np.linalg.norm(box[0]-box[3])
                # count area of the rectangle
                area = sideOne*sideTwo
                # find the largest rectangle within all contours
                if area > largestArea:
                    largestArea = area
                    largestRect = box
                    
                    
        if (largestArea > frameArea*0.02) & (Kirmizi_renk==0):
            # draw contour of the found rectangle on  the original image   
            cv2.drawContours(frame,[largestRect],0,(0,0,255),2)
            
            # cut and warp interesting area
            warped = four_point_transform(mask, [largestRect][0])
            			
	    # use function to detect the sign on the found rectangle
            detectedTrafficSign = identifyTrafficSign(warped)
            
            cv2.putText(frame, detectedTrafficSign, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            Tabela_kontrol=1
            
        else:
            Tabela_kontrol=0
            
        if (largestArea > frameArea*0.02) & (Kirmizi_renk==0) & (Tabela_kontrol==1):
            while Tabela_okundu==0 :
                # draw contour of the found rectangle on  the original image   
                cv2.drawContours(frame,[largestRect],0,(0,0,255),2)
            
                # cut and warp interesting area
                warped = four_point_transform(mask, [largestRect][0])
            			
	        # use function to detect the sign on the found rectangle
                detectedTrafficSign = identifyTrafficSign(warped)
            
                cv2.putText(frame, detectedTrafficSign, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                Tabela_1=1
                tabela=open("tabela.txt","r")
                Tabela=tabela.read()
                if Tabela=="1001":
                    Tabela_okundu=1
                    print("Tabela sağa algılandı===>>>>>>>")
                if Tabela=="0011":
                    Tabela_okundu=1
                    print("Tabela sola algılandı<<<<<=====")
                if Tabela=="0101":
                    Tabela_okundu=1
                    print("Tabela ileri algılandı...")
                if Tabela=="1011":
                    Tabela_okundu=1
                    print("Tabela geri algılandı...")

        if (largestArea > frameArea*0.02) & (Tabela_okundu==1) & (Kirmizi_renk==0):
            
            # draw contour of the found rectangle on  the original image   
            cv2.drawContours(frame,[largestRect],0,(0,0,255),2)
            
            # cut and warp interesting area
            warped = four_point_transform(mask, [largestRect][0])
            			
	    # use function to detect the sign on the found rectangle
            detectedTrafficSign = identifyTrafficSign(warped)
            
            cv2.putText(frame, detectedTrafficSign, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        else:
            Tabela_okundu=0
            

            
        if (largestArea > frameArea*0.02) & (Kirmizi_renk==0) & (Tabela_kontrol==1):
            
            while Tabela_okundu==0 :
                
                tabela=open("tabela.txt","r")
                Tabela=tabela.read()
                if Tabela=="1001":
                    Tabela_okundu=1
                    print("Tabela sağa algılandı===>>>>>>>")
                if Tabela=="0011":
                    Tabela_okundu=1
                    print("Tabela sola algılandı<<<<<=====")
                if Tabela=="0101":
                    Tabela_okundu=1
                    print("Tabela ileri algılandı...")
                if Tabela=="1011":
                    Tabela_okundu=1
                    print("Tabela geri algılandı...")

            

        if (Tabela_okundu==1) & (Kirmizi_renk==0) & (largestArea > frameArea*0.02) & (Tabela_kontrol==1):

            
            tabela=open("tabela.txt","r")
            Tabela=tabela.read()
            
            if Tabela=="0011":#sola dön
                                
                imgGrayscale=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
                imgBlurred = cv2.GaussianBlur(imgGrayscale,(5,5),0)
                imgCanny=cv2.Canny(imgBlurred,0,255)
                mask = np.zeros_like(imgCanny)
                vertices =np.array([[(5,240),(5,50),(125,50),(140,240)]],np.int32)
                cv2.fillPoly(mask, vertices, 255)
                masked = cv2.bitwise_and(imgCanny, mask)
                lines = cv2.HoughLinesP(masked,1,np.pi/180,10,maxLineGap=150)
                if lines is not None:
                    
                    sag=0
                    sol=0
                    for line in lines:
                        
                        x1, y1, x2, y2 = line[0]
                        size = math.hypot(x2 - x1, y2 - y1)
                        slope = ((y2-y1)/(x2-x1))  #egim
                        radius=math.sqrt(((y2-y1)*(y2-y1))+((x2-x1)*(x2-x1)))
                        if (slope > 0.5): #sag
                            
                            sag=1
                            if (size > largestRightLineSize):
                                largestRightLine = (x1, y1, x2, y2)                    
                            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        if (slope < -0.5): #sol
                            sol=1
                            if (size > largestLeftLineSize):
                                
                                largestLeftLine = (x1, y1, x2, y2)
                            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        if (sag==1) & (sol==1) & (radius>40):
                            
                            print('Tabela Araç sola gider====>>>>>>>')
                            solmotorsag.ChangeDutyCycle(0)
                            solmotorsol.ChangeDutyCycle(20)
                            sagmotorsag.ChangeDutyCycle(40)
                            sagmotorsol.ChangeDutyCycle(0)
                            time.sleep(0.1)
                        if (sag==1) & (sol==0) & (radius>40):
                            
                            print('Tabela Araç sola gider====>>>>>>>')
                            solmotorsag.ChangeDutyCycle(0)
                            solmotorsol.ChangeDutyCycle(20)
                            sagmotorsag.ChangeDutyCycle(40)
                            sagmotorsol.ChangeDutyCycle(0)
                            time.sleep(0.1)
                        if (sag==0) & (sol==1) & (radius>40):
                            
                            print('Tabela Araç sola gider====>>>>>>>')
                            solmotorsag.ChangeDutyCycle(0)
                            solmotorsol.ChangeDutyCycle(20)
                            sagmotorsag.ChangeDutyCycle(40)
                            sagmotorsol.ChangeDutyCycle(0)
                            time.sleep(0.1)
                if lines is None:
                    
                    print('Tabela Şerit algılanmadı...')
                    solmotorsag.ChangeDutyCycle(20)
                    solmotorsol.ChangeDutyCycle(0)
                    sagmotorsag.ChangeDutyCycle(0)
                    sagmotorsol.ChangeDutyCycle(20)
                    time.sleep(0.1)
               
               
            elif Tabela=="1001":#sağa dön
                                
                imgGrayscale=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
                imgBlurred = cv2.GaussianBlur(imgGrayscale,(5,5),0)
                imgCanny=cv2.Canny(imgBlurred,0,255)
                mask = np.zeros_like(imgCanny)
                vertices = np.array([[(180,240),(200,50),(310,50),(310,240)]],np.int32)
                cv2.fillPoly(mask, vertices, 255)
                masked = cv2.bitwise_and(imgCanny, mask)
                lines = cv2.HoughLinesP(masked,1,np.pi/180,10,maxLineGap=150)
                if lines is not None:
                    
                    sag=0
                    sol=0
                    for line in lines:
                        
                        x1, y1, x2, y2 = line[0]
                        size = math.hypot(x2 - x1, y2 - y1)
                        slope = ((y2-y1)/(x2-x1))
                        radius=math.sqrt(((y2-y1)*(y2-y1))+((x2-x1)*(x2-x1)))
                        if (slope > 0.5): #sag
                            
                            sag=1
                            if (size > largestRightLineSize):
                                
                                largestRightLine = (x1, y1, x2, y2)                    
                            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        if (slope < -0.5): #sol
                            
                            sol=1
                            if (size > largestLeftLineSize):
                                
                                largestLeftLine = (x1, y1, x2, y2)
                            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        
                        if (slope < -0.5): #sol
                            
                            sol=1
                            if (size > largestLeftLineSize):
                                
                                largestLeftLine = (x1, y1, x2, y2)
                            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        if (sag==1) & (sol==1) & (radius>40):
                            
                            print('Tabela Araç sağa gider====>>>>>>>')
                            solmotorsag.ChangeDutyCycle(0)
                            solmotorsol.ChangeDutyCycle(40)
                            sagmotorsag.ChangeDutyCycle(20)
                            sagmotorsol.ChangeDutyCycle(0)
                            time.sleep(0.1)
                        if (sag==1) & (sol==0) & (radius>40):
                            
                            print('Tabela Araç sağa gider====>>>>>>>')
                            solmotorsag.ChangeDutyCycle(0)
                            solmotorsol.ChangeDutyCycle(40)
                            sagmotorsag.ChangeDutyCycle(20)
                            sagmotorsol.ChangeDutyCycle(0)
                            time.sleep(0.1)
                        if (sag==0) & (sol==1) & (radius>40):
                            
                            print('Tabela Araç sağa gider====>>>>>>>')
                            solmotorsag.ChangeDutyCycle(0)
                            solmotorsol.ChangeDutyCycle(40)
                            sagmotorsag.ChangeDutyCycle(20)
                            sagmotorsol.ChangeDutyCycle(0)
                            time.sleep(0.1)
                if lines is None:
                    print('Tabela Şerit algılanmadı...')
                    solmotorsag.ChangeDutyCycle(20)
                    solmotorsol.ChangeDutyCycle(0)
                    sagmotorsag.ChangeDutyCycle(0)
                    sagmotorsol.ChangeDutyCycle(20)
                    time.sleep(0.1)
               
            elif Tabela=="1011":#geri dön
                                                
                imgGrayscale=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
                imgBlurred = cv2.GaussianBlur(imgGrayscale,(5,5),0)
                imgCanny=cv2.Canny(imgBlurred,0,255)
                mask = np.zeros_like(imgCanny)
                vertices =np.array([[(15,240),(95,140),(225,140),(305,240)]],np.int32)
                cv2.fillPoly(mask, vertices, 255)
                masked = cv2.bitwise_and(imgCanny, mask)
                lines = cv2.HoughLinesP(masked,1,np.pi/180,10,maxLineGap=150)
                if lines is not None:
                    
                    sag=0
                    sol=0
                    for line in lines:
                        
                        x1, y1, x2, y2 = line[0]
                        size = math.hypot(x2 - x1, y2 - y1)
                        slope = ((y2-y1)/(x2-x1))
                        radius=math.sqrt(((y2-y1)*(y2-y1))+((x2-x1)*(x2-x1)))
                        if (slope > 0.5): #sag
                            
                            sag=1
                            if (size > largestRightLineSize):
                                
                                largestRightLine = (x1, y1, x2, y2)                    
                            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        if (slope < -0.5): #sol
                            
                            sol=1
                            if (size > largestLeftLineSize):
                                
                                largestLeftLine = (x1, y1, x2, y2)
                            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        if (sag==1) & (sol==1) & (radius>40):
                            
                            print('Tabela Araç geri gider...')
                            solmotorsag.ChangeDutyCycle(20)
                            solmotorsol.ChangeDutyCycle(0)
                            sagmotorsag.ChangeDutyCycle(0)
                            sagmotorsol.ChangeDutyCycle(20)
                            time.sleep(0.1)
                        if (sag==1) & (sol==0) & (radius>40):
                            
                            print('Tabela Araç geri gider...')
                            solmotorsag.ChangeDutyCycle(20)
                            solmotorsol.ChangeDutyCycle(0)
                            sagmotorsag.ChangeDutyCycle(0)
                            sagmotorsol.ChangeDutyCycle(20)
                            time.sleep(0.1)
                        if (sag==0) & (sol==1) & (radius>40):
                            
                            print('Tabela Araç geri gider...')
                            solmotorsag.ChangeDutyCycle(20)
                            solmotorsol.ChangeDutyCycle(0)
                            sagmotorsag.ChangeDutyCycle(0)
                            sagmotorsol.ChangeDutyCycle(20)
                            time.sleep(0.1)
                        
                if lines is None:
                    
                    print('Tabela Şerit algılanmadı...')
                    solmotorsag.ChangeDutyCycle(20)
                    solmotorsol.ChangeDutyCycle(0)
                    sagmotorsag.ChangeDutyCycle(0)
                    sagmotorsol.ChangeDutyCycle(20)
                    time.sleep(0.1)
                            
            elif Tabela=="0101":#ileri git
                
                
                imgGrayscale=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
                imgBlurred = cv2.GaussianBlur(imgGrayscale,(5,5),0)
                imgCanny=cv2.Canny(imgBlurred,0,255)
                mask = np.zeros_like(imgCanny)
                vertices =np.array([[(15,240),(95,140),(225,140),(305,240)]],np.int32)
                cv2.fillPoly(mask, vertices, 255)
                masked = cv2.bitwise_and(imgCanny, mask)
                lines = cv2.HoughLinesP(masked,1,np.pi/180,10,maxLineGap=150)
                if lines is not None:
                    
                    sag=0
                    sol=0
                    for line in lines:
                        
                        x1, y1, x2, y2 = line[0]
                        size = math.hypot(x2 - x1, y2 - y1)
                        slope = ((y2-y1)/(x2-x1))
                        radius=math.sqrt(((y2-y1)*(y2-y1))+((x2-x1)*(x2-x1)))
                        if (slope > 0.5): #sag
                            
                            sag=1
                            if (size > largestRightLineSize):
                                
                                largestRightLine = (x1, y1, x2, y2)                    
                            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        if (slope < -0.5): #sol
                            sol=1
                            if (size > largestLeftLineSize):
                                largestLeftLine = (x1, y1, x2, y2)
                            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        if (sag==1) & (sol==1) & (radius>40):
                            
                            print('Tabela Araç düz gider...')
                            solmotorsag.ChangeDutyCycle(0)
                            solmotorsol.ChangeDutyCycle(20)
                            sagmotorsag.ChangeDutyCycle(20)
                            sagmotorsol.ChangeDutyCycle(0)
                            time.sleep(0.1)
                        if (sag==1) & (sol==0) & (radius>40):
                            
                            print('Tabela Araç düz gider...')
                            solmotorsag.ChangeDutyCycle(0)
                            solmotorsol.ChangeDutyCycle(20)
                            sagmotorsag.ChangeDutyCycle(20)
                            sagmotorsol.ChangeDutyCycle(0)
                            time.sleep(0.1)
                        if (sag==0) & (sol==1) & (radius>40):
                            print('Tabela Araç düz gider...')
                            solmotorsag.ChangeDutyCycle(0)
                            solmotorsol.ChangeDutyCycle(20)
                            sagmotorsag.ChangeDutyCycle(20)
                            sagmotorsol.ChangeDutyCycle(0)
                            time.sleep(0.1)
                        
                if lines is None:
                    print('Tabela Şerit algılanmadı...')
                    solmotorsag.ChangeDutyCycle(20)
                    solmotorsol.ChangeDutyCycle(0)
                    sagmotorsag.ChangeDutyCycle(0)
                    sagmotorsol.ChangeDutyCycle(20)
                    time.sleep(0.1)
            
               
        if Tabela_kontrol==0:
            #ŞERİT TAKİP
            imgGrayscale=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
            imgBlurred = cv2.GaussianBlur(imgGrayscale,(5,5),0)
            imgCanny=cv2.Canny(imgBlurred,0,255)
            mask = np.zeros_like(imgCanny)
            vertices =np.array([[(15,240),(95,140),(225,140),(305,240)]],np.int32)
            cv2.fillPoly(mask, vertices, 255)
            masked = cv2.bitwise_and(imgCanny, mask)
            lines = cv2.HoughLinesP(masked,1,np.pi/180,10,maxLineGap=150)
            
        if circles is not None:#kırmızı için
            
            print("Kırmızı renk algılandı...Araç durdu...")
            solmotorsag.ChangeDutyCycle(0)
            solmotorsol.ChangeDutyCycle(0)
            sagmotorsag.ChangeDutyCycle(0)
            sagmotorsol.ChangeDutyCycle(0)
            Kirmizi_renk=1
            for circle in circles[0]:
                
                x,y,radius=circle
                cv2.circle(frame,(x,y),3,(255,255,255),-1)
                cv2.circle(frame,(x,y),100,(0,0,255),3)
        if circless is not None:
            
            print("Yeşil renk algılandı...Araç için hareket...")
            Kirmizi_renk=0
            Yesil_renk=1
            for circle in circless[0]:#yeşil için
                
                x,y,radius=circle
                cv2.circle(frame,(x,y),3,(255,255,255),-1)
                cv2.circle(frame,(x,y),100,(0,255,0),3)
                

        if Yesil_renk==1: # Engel algılansada yeşilde hareket için
        
            if 20<mesafe_serit<55:
                Yesil_renk=1
            else:
                Yesil_renk=0
            
            
        #şerit için sabit değerler
        sag=0
        sol=0
        radius_sag=0
        radius_sol=0
        
        
        if (Kirmizi_renk==0) & (Tabela_kontrol==0):
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    size = math.hypot(x2 - x1, y2 - y1)
                    slope = ((y2-y1)/(x2-x1))
                    if (slope > 1): #sag
                        sag=1
                        radius=math.sqrt(((y2-y1)*(y2-y1))+((x2-x1)*(x2-x1)))
                        radius_sag=radius
                        if (size > largestRightLineSize):
                            largestRightLine = (x1, y1, x2, y2)                    
                        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    if (slope < -1): #sol
                        sol=1
                        radius=math.sqrt(((y2-y1)*(y2-y1))+((x2-x1)*(x2-x1)))
                        radius_sol=radius
                        if (size > largestLeftLineSize):
                            largestLeftLine = (x1, y1, x2, y2)
                        cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                   
                    if (sag==1) & (sol==1) & (radius_sag>40) & (radius_sol>40) & ((mesafe_serit>55) or (Yesil_renk==1)):
                        print('Şerit takip Araç düz gidiyor...',mesafe_serit)
                        solmotorsag.ChangeDutyCycle(0)
                        solmotorsol.ChangeDutyCycle(20)
                        sagmotorsag.ChangeDutyCycle(20)
                        sagmotorsol.ChangeDutyCycle(0)
                    if (sag==1) & (sol==0) & (radius_sag>40) & ((mesafe_serit>55) or (Yesil_renk==1)):
                        print('Şerit takip Araç sola gider<<<<<<<====',mesafe_serit)
                        solmotorsag.ChangeDutyCycle(0)
                        solmotorsol.ChangeDutyCycle(20)
                        sagmotorsag.ChangeDutyCycle(40)
                        sagmotorsol.ChangeDutyCycle(0)
                    if (sag==0) & (sol==1) & (radius_sol>40) & ((mesafe_serit>55) or (Yesil_renk==1)):
                        print('Şerit takip Araç sağa gider====>>>>>>>',mesafe_serit)
                        solmotorsag.ChangeDutyCycle(0)
                        solmotorsol.ChangeDutyCycle(40)
                        sagmotorsag.ChangeDutyCycle(20)
                        sagmotorsol.ChangeDutyCycle(0)
                    if (mesafe_serit<55) & (Yesil_renk==0):
                        print("Aracın önünde engel var...",mesafe_serit)
                        solmotorsag.ChangeDutyCycle(0)
                        solmotorsol.ChangeDutyCycle(0)
                        sagmotorsag.ChangeDutyCycle(0)
                        sagmotorsol.ChangeDutyCycle(0)
                        
            if lines is None:
                print('Şerit kontrol Şerit algılanmadı...')
                solmotorsag.ChangeDutyCycle(20)
                solmotorsol.ChangeDutyCycle(0)
                sagmotorsag.ChangeDutyCycle(0)
                sagmotorsol.ChangeDutyCycle(20)
                
        else:
            solmotorsag.ChangeDutyCycle(0)
            solmotorsol.ChangeDutyCycle(0)
            sagmotorsag.ChangeDutyCycle(0)
            sagmotorsol.ChangeDutyCycle(0)
        cv2.imshow("EKRAN", frame)
        rawCapture.truncate(0)
        if cv2.waitKey(1) & 0xFF is ord('q'):
            cv2.destroyAllWindows()
            print("Stop programm and close all windows")
            break
    solmotorsag.ChangeDutyCycle(0)
    solmotorsol.ChangeDutyCycle(0)
    sagmotorsag.ChangeDutyCycle(0)
    sagmotorsol.ChangeDutyCycle(0)
    solmotorsag.stop()
    solmotorsol.stop()
    sagmotorsag.stop()
    sagmotorsol.stop()
    
def identifyTrafficSign(image):
    SIGNS_LOOKUP = {
    (1, 0, 0, 1): 'SAGA DON', # turnRight
    (0, 0, 1, 1): 'SOLA DON', # turnLeft
    (0, 1, 0, 1): 'ILERI GIT', # moveStraight
    (1, 0, 1, 1): 'GERI DON', # turnBack
    }

    THRESHOLD = 150
    image = cv2.bitwise_not(image)
    (subHeight, subWidth) = np.divide(image.shape, 10)
    subHeight = int(subHeight)
    subWidth = int(subWidth)

    # mark the ROIs borders on the image
    cv2.rectangle(image, (subWidth, 4*subHeight), (3*subWidth, 9*subHeight), (0,255,0),2) # left block
    cv2.rectangle(image, (4*subWidth, 4*subHeight), (6*subWidth, 9*subHeight), (0,255,0),2) # center block
    cv2.rectangle(image, (7*subWidth, 4*subHeight), (9*subWidth, 9*subHeight), (0,255,0),2) # right block
    cv2.rectangle(image, (3*subWidth, 2*subHeight), (7*subWidth, 4*subHeight), (0,255,0),2) # top block

    # substract 4 ROI of the sign thresh image
    leftBlock = image[4*subHeight:9*subHeight, subWidth:3*subWidth]
    centerBlock = image[4*subHeight:9*subHeight, 4*subWidth:6*subWidth]
    rightBlock = image[4*subHeight:9*subHeight, 7*subWidth:9*subWidth]
    topBlock = image[2*subHeight:4*subHeight, 3*subWidth:7*subWidth]

    # we now track the fraction of each ROI
    leftFraction = np.sum(leftBlock)/(leftBlock.shape[0]*leftBlock.shape[1])
    centerFraction = np.sum(centerBlock)/(centerBlock.shape[0]*centerBlock.shape[1])
    rightFraction = np.sum(rightBlock)/(rightBlock.shape[0]*rightBlock.shape[1])
    topFraction = np.sum(topBlock)/(topBlock.shape[0]*topBlock.shape[1])

    segments = (leftFraction, centerFraction, rightFraction, topFraction)
    segments = tuple(1 if segment > THRESHOLD else 0 for segment in segments)
    
    if segments in SIGNS_LOOKUP:
        tabela=open("tabela.txt","w")
        if (segments==(1, 0, 0, 1)):
            tabela.write("1001")
        elif (segments==(0, 0, 1, 1)):
            tabela.write("0011")
        elif (segments==(0, 1, 0, 1)):
            tabela.write("0101")
        elif (segments==(1, 0, 1, 1)):
            tabela.write("1011")
        else:
            return SIGNS_LOOKUP[segments]
    else:
        return None
    cv2.imshow("Warped", image)

    
if __name__ == '__main__':
    
    main()
io.cleanup()
