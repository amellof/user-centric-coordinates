import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;
import javax.vecmath.Matrix3f;
import java.util.*;
import com.badlogic.gdx.math.*;
import java.util.ArrayList;


public class coordinatesConversion{

	private float[] worldAcc;
	private float[] userAcc;

	public coordinatesConversion(){
		this.worldAcc = new float[3];
		this.userAcc = new float[3];
	}

	/**
	* Get the world coordinates
	*/
	float[] getWorldAcc(){
		return worldAcc;
	}

	/**
	* Get the world coordinates
	*/
	float[] getUserAcc(){
		return userAcc;
	}

	/**
	* Calculate the world coordinates given an acceleration sample and 
	* the corresponding rotation vector sample
	*/
	public void calculateWorldCoordinates(float[] acc, float[] rotationVector) {
		// transform the rotation vector sample in a quaternion form
		Quaternion quaternion = new Quaternion(rotationVector[0], rotationVector[1], rotationVector[2], rotationVector[3]);
		
		// get the conjugate of this quaternion
		Quaternion conjQuaternion = new Quaternion(quaternion);
		conjQuaternion.conjugate();
		
		// format the accelerometer sample in a quaternion in order to perform the operations
		Quaternion vector = new Quaternion(acc[0], acc[1], acc[2], 0);
		
		// perform the rotation by multiplying the quaternion and the accelerometer sample, then the result and the quaternion conjugate
		Quaternion quatMultiplication = new Quaternion(quaternion);		
		quatMultiplication.mul(vector);
		quatMultiplication.mul(conjQuaternion);
		
		// get the accelerations rotated
		this.worldAcc = {quatMultiplication.x, quatMultiplication.y, quatMultiplication.z};
	}

	/**
	* Get the coefficients of a hamming window with lenght m
	*/
	public static float[] windowHamming(final float[] fir) {
		final int m = fir.length - 1;
		for (int i = 0; i < fir.length; i++) {
			fir[i] = (float) (0.54 - 0.46 * MathUtils.cos(2 * MathUtils.PI * i / m));
		}
		return fir;
	}
	
	/** 
	* Get the coefficients of a gaussian window with lenght m and standard deviation std
	*/
	public static float[] windowGaussian(final float[] fir, float std) {
		float n;
		final int m = fir.length - 1;
		for (int i = 0; i < fir.length; i++) {
			n = i - ((m - 1) / 2);
			fir[i] = (float) Math.exp((-1/2) * Math.pow((n/std), 2));
		}
		return fir;
	}
	
	/** 
	* Perform the convolution between two vectors 	
	*/
	public static float[] convolve(float[] h, float[] x) {
		int M = h.length;
		int N = x.length;
		float[] convolution = new float[N + M - 1];
		for (int n = 0; n < convolution.length; n++) {
			for (int k = 0, l = n - k; k < M; k++, l = n - k) {
				if (l >= 0 && l < N) {
					convolution[n] += h[k] * x[l];
				}
			}
		}
		return convolution;
	}

	/** 
	* Apply a filter by the convolution of the filter coefficients	
	*/
	public static float[] applyFilter(float[] acc, float[] coeffs) {
    	float[] result = convolve(coeffs, acc);
    	return result;
	}

	/**
	* Apply a bandstop filter of a given frequency in a temporal vector acc collected in a
	* sampling frequency Fs. The filter is applied in each axis singly
	*/
	public static float[] notchFilter(float Fs, float frequency, float[] acc) {
		// obtain the filter coefficients according the bandstop frequency and the sampling frequency
		float w = (2 * MathUtils.PI * frequency) / Fs;
		float r0 = 1.0f;
		float rp = 0.9f;
		float realZ0 = r0*(MathUtils.cos(w));
		float imZ0 = r0*(MathUtils.sin(w));
		float realZp = rp*(MathUtils.cos(w));
		float imZp = rp*(MathUtils.sin(w));
		
		float[] coeffs0 = {1, -2*realZ0, (realZ0*realZ0)+(imZ0*imZ0)};
		float[] coeffsP = {1, -2*realZp, (realZp*realZp)+(imZp*imZp)};

		float[] acc_filter = new float[acc.length];
		
		// the filter order is two, so it is necessary to initialize the two first samples of the temporal vector
		int k = 2;
		acc_filter[0] = acc[0];
		acc_filter[1] = (coeffs0[0]*acc[1] + coeffs0[1]*acc[0] - coeffsP[1]*acc_filter[0])/coeffsP[0];
		
		// the filter is applied in each of the other samples
		for(int i = k; i < acc.length; i++) {
			acc_filter[i] = (coeffs0[0]*acc[i] + coeffs0[1]*acc[i-1] + coeffs0[2]*acc[i-2] - coeffsP[1]*acc_filter[i-1] - coeffsP[2]*acc_filter[i-2])/coeffsP[0];
		}
		return acc_filter;
	}

	/**
	* The velocities are obtained given a temporal vector of the triaxial acceleration data
	*/
	public static float[][] calculateVelocities(float[][] vecAcc_w, float deltaT) {
	    float[][] velocities = new float[vecAcc_w.length][vecAcc_w[0].length];

	    // it is adopted the trapezoidal rule for the velocities computation
	    for(int i = 1; i < vecAcc_w.length; i++) {
		    for(int j = 0; j < vecAcc_w[0].length; j++) {
			    velocities[i][j] = velocities[i-1][j] + (deltaT * vecAcc_w[i-1][j]) + ((deltaT/2) * (vecAcc_w[i][j] - vecAcc_w[i-1][j]));
		    }
	    }
	    return velocities;
	}

	/** 
	* Get the user coordinates for an acceleration sample acc in world coordinates 
	* given its velocity values
	*/
	private static float[] calculateUserCoordinates(float[] acc, float[] velocity) {
		// the first column vector corresponds to the main direction of movement given by the normalized velocities 
		Vector3f e_1 = new Vector3f(velocity);
		e_1.normalize();
		
		// the second column vector is orthogonal to the first one
		Vector3f e_2 = new Vector3f(0, 0, 1);
		e_2.cross(e_1, e_2);
		e_2.normalize();
		
		// the third column vector is orthogonal to both 
		Vector3f e_3 = new Vector3f();
		e_3.cross(e_1, e_2);
		
		// the rotation matrix is formed for the three vector columns
		Matrix3f mat = new Matrix3f();
		mat.setColumn(0, e_1);
		mat.setColumn(1, e_2);
		mat.setColumn(2, e_3);
		
		// given the acceleration sample, the user-centric coordinates are obtained through the rotation matrix
		Vector3f acc_w = new Vector3f(acc);
		Vector3f acc_u = new Vector3f();
		mat.transform(acc_w, acc_u);
		this.userAcc = {acc_u.x, acc_u.y, acc_u.z};		
	}
}