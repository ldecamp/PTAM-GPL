# DO NOT DELETE THIS LINE -- make depend depends on it.
# Edit the lines below to point to any needed include and link paths
# Or to change the compiler's optimization flags
CC = g++
COMPILEFLAGS =  -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3 `pkg-config opencv --cflags`
LINKFLAGS =  -lGVars3 -lcvd -llapack -lGL -lGLU -lglut `pkg-config opencv --libs` 

BINOUTPUT="bin"
# Edit this line to change video source
#DV or V4L or Gstreamer_File or OpenCV
VIDEOSOURCE = VideoSource_Linux_OpenCV.o

OBJECTS=	main.o\
		GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		System.o \
		ATANCamera.o\
		KeyFrame.o\
		MapPoint.o\
		Map.o\
		SmallBlurryImage.o\
		ShiTomasi.o \
		HomographyInit.o \
		MapMaker.o \
		Bundle.o \
		PatchFinder.o\
		Relocaliser.o\
		MiniPatch.o\
		MapViewer.o\
		ARDriver.o\
		EyeGame.o\
		Tracker.o

CALIB_OBJECTS=	GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		CalibImage.o \
		CalibCornerPatch.o\
		ATANCamera.o \
		CameraCalibrator.o

All: PTAM CameraCalibrator

PTAM: $(OBJECTS)
	$(CC) -o $(BINOUTPUT)/PTAM $(OBJECTS) $(LINKFLAGS)

CameraCalibrator:$(CALIB_OBJECTS)
	$(CC) -o $(BINOUTPUT)/CameraCalibrator $(CALIB_OBJECTS) $(LINKFLAGS)
	cp calibrator_settings.cfg bin/calibrator_settings.cfg


%.o: %.cc
	$(CC) $< -o $@ -c $(COMPILEFLAGS)

clean:
	rm -rf *.o
	rm -rf $(BINOUTPUT)/*

depend:
	rm dependecies; touch dependencies
	makedepend -fdependencies $(INCLUDEFLAGS) $(MOREINCS) *.cc *.h


-include dependencies