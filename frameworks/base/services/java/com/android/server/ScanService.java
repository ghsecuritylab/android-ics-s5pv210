package com.android.server;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.util.Slog;
import com.android.server.IScanService;

public class ScanService extends IScanService.Stub {
	private static final String TAG = "ScanService";
	private static final boolean DEBUG = true;
	private final Context mContext;
	private int mGpsFlag = -1;
	private int mPtr = 0;

	private final BroadcastReceiver mBroadcastReciever = new BroadcastReceiver() {
	    @Override public void onReceive(Context context, Intent intent) {
	        String action = intent.getAction();

	        if (action.equals("vanstone.gps.start")) {
	             if (DEBUG) Slog.d(TAG, "gps.start");
	           mGpsFlag = 1;
	        } else if (action.equals("vanstone.gps.stop")) {
	            if (DEBUG) Slog.d(TAG, "gps.stop");
	            mGpsFlag = 0;
	         }
	    }
	};

	ScanService(Context context) {
		mContext = context;
		mPtr = init_native();
		
		if(mPtr == 0) {
			Slog.e(TAG, "Failed to initialize Scan service.");
		}

		IntentFilter intentFilter = new IntentFilter();
		intentFilter.addAction("vanstone.gps.start");
		intentFilter.addAction("vanstone.gps.stop");
		context.registerReceiver(mBroadcastReciever, intentFilter);
	}

	public String getBarcode() {
		String code = null;
		Intent intentStart = new Intent();
		Intent intentStop = new Intent();
		if(mPtr == 0) {
			Slog.e(TAG, "Scan service is not initialized.");
			return "Scan service is not initialized.";
		}

		if(mGpsFlag == 1){
			intentStart.setAction("vanstone.scan.start");
			mContext.sendBroadcast(intentStart);
	        	try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			code = getBarcode_native(mPtr);
			intentStop.setAction("vanstone.scan.stop");
			mContext.sendBroadcast(intentStop);
		}
		else{
			code = getBarcode_native(mPtr);
		}
		return code;
	}
	
	private static native int init_native();
	private static native String getBarcode_native(int ptr);
};
