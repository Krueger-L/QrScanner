package imp.qrscanner;

import android.app.Activity;
import android.graphics.Bitmap;
import android.hardware.Camera;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.TextView;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;


public class MainActivity extends Activity {

    private Camera mCamera;
    private CameraFeed mPreview;
    private ImageProcessor iProcessor =  new ImageProcessor();

    // implements opencvManager
    private BaseLoaderCallback mOpenCVCallBack = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    //Log.i(TAG, "OpenCV loaded successfully");

                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };



    // do this when the application starts up
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Create an instance of Camera
        mCamera = getCameraInstance();
        // Create our Preview view and set it as the content of our activity.
        mPreview = new CameraFeed(this, mCamera);
        FrameLayout preview = (FrameLayout) findViewById(R.id.CameraFeed);
        preview.addView(mPreview);

        // implement button listener
        final Button button = (Button) findViewById(R.id.button_id);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                // the only directive Imageprocessor needs to process the image
                iProcessor.setOriginalMat(
                        mPreview.getOriginalFrame(),
                        mPreview.getPreviewW(),
                        mPreview.getPreviewH());


                // display DebugView
                Bitmap bm = Bitmap.createBitmap(
                        iProcessor.getdebugMat().cols(),
                        iProcessor.getdebugMat().rows(),
                        Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(iProcessor.getdebugMat(), bm);
                ImageView iv = (ImageView) findViewById(R.id.imageView2);
                iv.setImageBitmap(bm);

            }
        });
    }

    // release camera on paused activity
    @Override
    protected void onPause(){
        super.onPause();
        if(mCamera != null) {
            mCamera.release();
            mCamera = null;
        }
    }

    // get camera and refresh CameraFeed + opencvmanager reconnect
    @Override
    protected void onResume(){
        super.onResume();
        if(mCamera == null){
            mCamera = getCameraInstance();
            mPreview.refreshCamera(mCamera);
        }
        if (!OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mOpenCVCallBack))
        {
            //Log.e(TAG, "Cannot connect to OpenCV Manager");
        }
    }

    // safe way to get camera instance
    public static Camera getCameraInstance(){
        Camera c = null;
        try {
            c = Camera.open();
        }
        catch (Exception e){
            // Camera is not available
        }
        return c; // returns null if camera is unavailable
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }
}
