package com.arjo129.facepulse;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.PorterDuff;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.support.annotation.NonNull;
import android.support.annotation.RequiresApi;
import android.support.v4.app.ActivityCompat;
import android.support.v4.util.Pair;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.util.Size;
import android.util.SparseIntArray;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.widget.Toast;

import com.google.android.gms.tasks.OnFailureListener;
import com.google.android.gms.tasks.OnSuccessListener;
import com.google.android.gms.tasks.Task;
import com.google.firebase.FirebaseApp;
import com.google.firebase.ml.vision.FirebaseVision;
import com.google.firebase.ml.vision.common.FirebaseVisionImage;
import com.google.firebase.ml.vision.common.FirebaseVisionImageMetadata;
import com.google.firebase.ml.vision.common.FirebaseVisionPoint;
import com.google.firebase.ml.vision.face.FirebaseVisionFace;
import com.google.firebase.ml.vision.face.FirebaseVisionFaceDetector;
import com.google.firebase.ml.vision.face.FirebaseVisionFaceDetectorOptions;
import com.google.firebase.ml.vision.face.FirebaseVisionFaceLandmark;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class PulseFace extends AppCompatActivity {

    class FacePulse {
        public Rect boundingbox;
        public String text;
    };
    private static final String TAG = "com.arjo129.pulseface";
    private TextureView textureView;
    private SurfaceView surfaceView;
    private Size mPreviewSize;
    private static final SparseIntArray ORIENTATIONS = new SparseIntArray();
    private Queue<FacePulse> queue;
    static {
        ORIENTATIONS.append(Surface.ROTATION_0, 90);
        ORIENTATIONS.append(Surface.ROTATION_90, 0);
        ORIENTATIONS.append(Surface.ROTATION_180, 270);
        ORIENTATIONS.append(Surface.ROTATION_270, 180);
    }
    private String cameraId;
    protected CameraDevice cameraDevice;
    boolean newFaceAvailable = false;

    private final CameraDevice.StateCallback stateCallback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(CameraDevice camera) {
            //This is called when the camera is open
            Log.e(TAG, "onOpened");
            cameraDevice = camera;
            createCameraPreview();
        }
        @Override
        public void onDisconnected(CameraDevice camera) {
            cameraDevice.close();
        }
        @Override
        public void onError(CameraDevice camera, int error) {
            cameraDevice.close();
            cameraDevice = null;
        }
    };
    protected CameraCaptureSession cameraCaptureSessions;
    protected CaptureRequest captureRequest;
    protected CaptureRequest.Builder captureRequestBuilder;
    private Size imageDimension;

    private static final int REQUEST_CAMERA_PERMISSION = 200;
    private ImageReader imageReader;

    private Handler mBackgroundHandler;
    private HandlerThread mBackgroundThread;

    private HashMap<Integer, List<Pair<Long, Float>>> people;

    private final TextureView.SurfaceTextureListener textureListener = new TextureView.SurfaceTextureListener() {
        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
            //open your camera here
            openCamera();
        }
        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
            // Transform you image captured size according to the surface width and height

        }
        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
            return false;
        }
        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        }
    };;

    FirebaseVisionFaceDetectorOptions realTimeOpts = null;

    FirebaseVisionFaceDetector detector = null;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        queue = new LinkedList<>();
        FirebaseApp.initializeApp(getApplicationContext());
        setContentView(R.layout.activity_pulse_face);
        textureView = (TextureView) findViewById(R.id.texture);
        textureView.setSurfaceTextureListener(textureListener);
        people = new HashMap<>();
        realTimeOpts =
                new FirebaseVisionFaceDetectorOptions.Builder()
                        .setContourMode(FirebaseVisionFaceDetectorOptions.NO_CONTOURS)
                        .setLandmarkMode(FirebaseVisionFaceDetectorOptions.ALL_LANDMARKS)
                        .setClassificationMode(FirebaseVisionFaceDetectorOptions.NO_CLASSIFICATIONS)
                        .setPerformanceMode(FirebaseVisionFaceDetectorOptions.FAST)
                        .enableTracking()
                        .build();
        detector = FirebaseVision.getInstance()
                .getVisionFaceDetector(realTimeOpts);

        surfaceView = (SurfaceView) findViewById(R.id.surfaceView);
        surfaceView.setZOrderOnTop(true);

        final SurfaceHolder mHolder = surfaceView.getHolder();
        mHolder.setFormat(PixelFormat.TRANSPARENT);
        mHolder.addCallback(new SurfaceHolder.Callback() {
            AsyncTask asyncTask;
            @Override
            public void surfaceCreated(SurfaceHolder holder) {
                 asyncTask = new AsyncTask() {
                    private int pos = 20;
                    @Override
                    protected Object doInBackground(Object[] objects) {
                        while(true) {
                            // Lock canvas for drawing
                            Canvas c = mHolder.lockCanvas(null);

                            synchronized (mHolder) {
                                // First draw off screen bitmap to off screen canvas one line down
                                if(newFaceAvailable) {
                                    c.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);
                                    newFaceAvailable = false;
                                }
                                if(!queue.isEmpty()) {

                                    while(!queue.isEmpty()) {
                                        Paint brush = new Paint();
                                        brush.setColor(Color.GREEN);
                                        brush.setStrokeWidth(10);
                                        brush.setStyle(Paint.Style.STROKE);
                                        Rect rect = queue.peek().boundingbox;
                                        //c.drawRect(rect,brush);
                                        brush.setStrokeWidth(2);
                                        brush.setTextSize(70);
                                        c.drawText(queue.peek().text,rect.left,rect.bottom+30,brush);
                                        queue.remove();
                                    }
                                }
                                // Other drawing to canvas comes here

                            }
                            try {
                                Thread.sleep(100);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            mHolder.unlockCanvasAndPost(c);
                        }
                    }
                };
                asyncTask.execute();
                //rn.run();
            }

            @Override
            public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

            }

            @Override
            public void surfaceDestroyed(SurfaceHolder holder) {
                asyncTask.cancel(true);
            }
        });





    }


    boolean imageLock = false;
    ImageReader.OnImageAvailableListener imageListener = new ImageReader.OnImageAvailableListener() {
        @Override
        public void onImageAvailable(ImageReader reader) {

            final Image image = reader.acquireLatestImage();
            final Long time = System.currentTimeMillis();
            if(imageLock) {
                image.close();
                return;
            }
            imageLock = true;
            //Log.d(TAG,"Got image");

           try {
                final FirebaseVisionImage fbImage = FirebaseVisionImage.fromMediaImage(image, getRotationCompensation(cameraId,
                        (Activity) PulseFace.this, PulseFace.this));
                Task<List<FirebaseVisionFace>> result =
                        detector.detectInImage(fbImage)
                                .addOnSuccessListener(
                                        new OnSuccessListener<List<FirebaseVisionFace>>() {
                                            @Override
                                            public void onSuccess(List<FirebaseVisionFace> faces) {
                                                long t= System.currentTimeMillis();
                                                processFaces(faces, image, time);
                                                Log.d(TAG, "Face processing took "+ (System.currentTimeMillis() - t));
                                                Log.d(TAG, "Google took "+ (t - time));
                                                image.close();
                                                imageLock = false;
                                            }
                                        })
                                .addOnFailureListener(
                                        new OnFailureListener() {
                                            @Override
                                            public void onFailure(@NonNull Exception e) {
                                                // Task failed with an exception
                                                // ...
                                                Log.e(TAG, "Firebase failed "+e);
                                                image.close();
                                                imageLock = false;
                                            }
                                        });
            } catch (CameraAccessException ca) {
                Log.d(TAG,"Failed to get camera!!");
                image.close();
            }

            //Log.d(TAG,"Got image");
        }

    };

    int[] yuvToRgb(int y, int u, int v) {
        int[] rgb = new int[3];
        float Yf = 1.164f*((float)y) - 16.0f;
        int R = (int)(Yf + 1.596f*v);
        int G = (int)(Yf - 0.813f*v - 0.391f*u);
        int B = (int)(Yf            + 2.018f*u);

        // Clip rgb values to 0-255
        rgb[0] = R < 0 ? 0 : R > 255 ? 255 : R;
        rgb[1] = G < 0 ? 0 : G > 255 ? 255 : G;
        rgb[2] = B < 0 ? 0 : B > 255 ? 255 : B;

        return rgb;
    }

    byte shiftIfOdd(byte b, int i) {
        if(i%2 == 1){
            return (byte) (b & 0xf0);
        } else {
            return (byte) (b & 0x0f);
        }
    }

    float greenSum(Image image, int left,int top, int right, int bottom){
        int green = 0, count = 0;
        Image.Plane Y = image.getPlanes()[0];
        Image.Plane U = image.getPlanes()[1];
        Image.Plane V = image.getPlanes()[2];
        ByteBuffer byteBufferY = Y.getBuffer();
        ByteBuffer byteBufferU = U.getBuffer();
        ByteBuffer byteBufferV = V.getBuffer();
        for(int j = top ; j < bottom; j++) {
            for (int i = left + j * Y.getRowStride(); i < right + j * Y.getRowStride() && i < Y.getBuffer().capacity(); i++) {
                int[] rgb = yuvToRgb(byteBufferY.get(i), shiftIfOdd(byteBufferU.get(i/2), i), shiftIfOdd(byteBufferV.get(i/2), i));
                green += rgb[0];
                count++;
            }
        }
        return (float)green/(float)count;
    }
    protected void processFaces(List<FirebaseVisionFace> faces, Image image, Long time) {
        newFaceAvailable = true;
        for (FirebaseVisionFace face : faces) {

            FacePulse facePulse = new FacePulse();
            Rect bounds = face.getBoundingBox();
            facePulse.boundingbox = bounds;

            float rotY = face.getHeadEulerAngleY();  // Head is rotated to the right rotY degrees
            float rotZ = face.getHeadEulerAngleZ();  // Head is tilted sideways rotZ degrees

            // If landmark detection was enabled (mouth, ears, eyes, cheeks, and
            // nose available):
            FirebaseVisionPoint leftEyePos = null, rightEyePos = null;
            FirebaseVisionFaceLandmark leftEye = face.getLandmark(FirebaseVisionFaceLandmark.LEFT_EYE);
            if (leftEye != null) {
                leftEyePos = leftEye.getPosition();
            }

            FirebaseVisionFaceLandmark rightEye = face.getLandmark(FirebaseVisionFaceLandmark.RIGHT_EYE);
            if (leftEye != null) {
                rightEyePos = rightEye.getPosition();
            }

            float greensum = -1;
            if (leftEyePos != null && rightEyePos != null) {
                int leftX = Math.max(Math.round(leftEyePos.getX()),0);
                int leftY = Math.max(Math.round(leftEyePos.getY()),0);
                int rightX = Math.max(Math.round(rightEyePos.getX()),0);
                int rightY = Math.max(Math.round(rightEyePos.getY()),0);
                greensum = greenSum(image, leftX,Math.max(leftY-60,0),(int)rightX,(int)rightY);

            }


            if (face.getTrackingId() != FirebaseVisionFace.INVALID_ID && greensum > 0) {
                int id = face.getTrackingId();
                facePulse.text = "Calculating";
                if(people.containsKey(id)) {
                    people.get(id).add(new Pair<Long, Float>(time, greensum));
                    Log.d(TAG,"New info for face: ("+id+","+time+","+greensum+")");
                    Long start = 0l, end = 0l;
                    if(people.get(id).size() > 10) {
                        int count = 0;
                        start = people.get(id).get(0).first;
                        end = people.get(id).get(people.get(id).size() - 1).first;
                        long duration  = end - start;
                        float avg = 0;
                        for(Pair<Long, Float> values: people.get(id)) {
                            avg += values.second;
                        }
                        avg /= people.get(id).size();
                        float var = 0;
                        for(Pair<Long, Float> values: people.get(id)) {
                            var += (values.second - avg)*(values.second - avg);
                        }
                        var /= people.get(id).size();
                        boolean prevDir = (people.get(id).get(1).second - people.get(id).get(0).second) > 0;
                        for(int i = 2; i < people.get(id).size(); i++){
                            boolean currDir = (people.get(id).get(i).second - people.get(id).get(i-1).second) > 0;
                            if(currDir != prevDir) count++;
                            prevDir = currDir;
                        }
                        facePulse.text = "" +  (Math.round(count/(duration/60000.0f))+20) + " Bpm";
                        Log.d(TAG, "id: " + id +"BPM: "+ facePulse.text + "Var: "+var + " sample" + duration/people.get(id).size() );
                    }
                } else {
                    List<Pair<Long, Float>> pulse = new ArrayList<>();
                    pulse.add(new Pair(time, greensum));
                    people.put(id, pulse);

                }
                queue.add(facePulse);
            }
        }

    }
    protected void createCameraPreview() {
        try {
            SurfaceTexture texture = textureView.getSurfaceTexture();
            assert texture != null;
            texture.setDefaultBufferSize(imageDimension.getWidth(), imageDimension.getHeight());
            Surface surface = new Surface(texture);
            captureRequestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            captureRequestBuilder.addTarget(surface);
            imageReader = ImageReader.newInstance(textureView.getWidth(), textureView.getHeight(), ImageFormat.YUV_420_888, 2);
            imageReader.setOnImageAvailableListener(imageListener, mBackgroundHandler);
            captureRequestBuilder.addTarget(imageReader.getSurface());
            List<Surface> outputSurfaces = new ArrayList<>();
            outputSurfaces.add(surface);
            outputSurfaces.add(imageReader.getSurface());
            cameraDevice.createCaptureSession(outputSurfaces, new CameraCaptureSession.StateCallback(){
                @Override
                public void onConfigured(@NonNull CameraCaptureSession cameraCaptureSession) {
                    //The camera is already closed
                    if (null == cameraDevice) {
                        return;
                    }
                    // When the session is ready, we start displaying the preview.
                    cameraCaptureSessions = cameraCaptureSession;
                    updatePreview();
                }
                @Override
                public void onConfigureFailed(@NonNull CameraCaptureSession cameraCaptureSession) {
                    Toast.makeText(PulseFace.this, "Configuration change", Toast.LENGTH_SHORT).show();
                }
            }, null);
            Log.d(TAG, "Created new camera preview");
        } catch (CameraAccessException e) {
            Log.d(TAG, "Camera Access Exception" + e);
        }
    }

    protected void startBackgroundThread() {
        mBackgroundThread = new HandlerThread("Camera Background");
        mBackgroundThread.start();
        mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
    }

    protected void stopBackgroundThread() {
        mBackgroundThread.quitSafely();
        try {
            mBackgroundThread.join();
            mBackgroundThread = null;
            mBackgroundHandler = null;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void openCamera() {
        CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        Log.e(TAG, "is camera open");
        try {
            cameraId = manager.getCameraIdList()[0];
            CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
            StreamConfigurationMap map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            assert map != null;
            imageDimension = map.getOutputSizes(SurfaceTexture.class)[0];
            // Add permission for camera and let user grant the permission
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA, Manifest.permission.WRITE_EXTERNAL_STORAGE}, REQUEST_CAMERA_PERMISSION);
                return;
            }
            manager.openCamera(cameraId, stateCallback, null);
        } catch (CameraAccessException e) {
            e.printStackTrace();
            Log.e(TAG, "Camera Access exception " + e);

        }
        Log.e(TAG, "openCamera X");
    }



    protected void updatePreview() {
        if(null == cameraDevice) {
            Log.e(TAG, "updatePreview error, return");
        }
        captureRequestBuilder.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
        try {
            cameraCaptureSessions.setRepeatingRequest(captureRequestBuilder.build(), null, mBackgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    private void closeCamera() {
        if (null != cameraDevice) {
            cameraDevice.close();
            cameraDevice = null;
        }
    }

    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        if (requestCode == REQUEST_CAMERA_PERMISSION) {
            if (grantResults[0] == PackageManager.PERMISSION_DENIED) {
                // close the app
                Toast.makeText(this, "Sorry!!!, you can't use this app without granting permission", Toast.LENGTH_LONG).show();
                finish();
            }
        }
    }
    @Override
    protected void onResume() {
        super.onResume();
        Log.e(TAG, "onResume");
        startBackgroundThread();
        if (textureView.isAvailable()) {
            openCamera();
        } else {
            textureView.setSurfaceTextureListener(textureListener);
        }


    }
    @Override
    protected void onPause() {
        Log.e(TAG, "onPause");
        stopBackgroundThread();
        closeCamera();
        super.onPause();
    }
    /**
     * Get the angle by which an image must be rotated given the device's current
     * orientation.
     */
    @RequiresApi(api = Build.VERSION_CODES.LOLLIPOP)
    private int getRotationCompensation(String cameraId, Activity activity, Context context)
            throws CameraAccessException {
        // Get the device's current rotation relative to its "native" orientation.
        // Then, from the ORIENTATIONS table, look up the angle the image must be
        // rotated to compensate for the device's rotation.
        int deviceRotation = activity.getWindowManager().getDefaultDisplay().getRotation();
        int rotationCompensation = ORIENTATIONS.get(deviceRotation);

        // On most devices, the sensor orientation is 90 degrees, but for some
        // devices it is 270 degrees. For devices with a sensor orientation of
        // 270, rotate the image an additional 180 ((270 + 270) % 360) degrees.
        CameraManager cameraManager = (CameraManager) context.getSystemService(CAMERA_SERVICE);
        int sensorOrientation = cameraManager
                .getCameraCharacteristics(cameraId)
                .get(CameraCharacteristics.SENSOR_ORIENTATION);
        rotationCompensation = (rotationCompensation + sensorOrientation + 270) % 360;

        // Return the corresponding FirebaseVisionImageMetadata rotation value.
        int result;
        switch (rotationCompensation) {
            case 0:
                result = FirebaseVisionImageMetadata.ROTATION_0;
                break;
            case 90:
                result = FirebaseVisionImageMetadata.ROTATION_90;
                break;
            case 180:
                result = FirebaseVisionImageMetadata.ROTATION_180;
                break;
            case 270:
                result = FirebaseVisionImageMetadata.ROTATION_270;
                break;
            default:
                result = FirebaseVisionImageMetadata.ROTATION_0;
                Log.e(TAG, "Bad rotation value: " + rotationCompensation);
        }
        return result;
    }



}
