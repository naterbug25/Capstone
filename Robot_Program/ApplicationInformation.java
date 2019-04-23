/*
 * Author: Nathan Huber
 * Purpose: Background Program Handling for Two Robot cobot control
 * Date Started: 20190205
 * Date of Rev: 20190205
 * Revision: 1.0
 * 
 * ---Revision History---
 * Date			Author			Rev			Purpose
 * 20190205		Nathan Huber	1.0			Initial Program
 * 
 * 
 * 
 * 
 */
package Capstone_Huber;

import com.kuka.roboticsAPI.controllerModel.recovery.IRecovery;
import com.kuka.roboticsAPI.geometricModel.Frame;

public class ApplicationInformation implements IApplicationInformationFunction {
	Frame		Get_Robot_Pos_Frame;
	double		Get_Force_X_Offset;
	double		Get_Force_Y_Offset;
	double		Get_Force_Z_Offset;
	int			Get_Force_Avg_Comp;

	boolean		_assembly;						// Demo
	IRecovery	_applicationRecoveryInterface;	// Demo

	/*********************************************************************************************
	 * SET Objects
	 *********************************************************************************************/
	// Store current position of robot
	public void setRobotPosition(Frame Robot_Pos_Frame) {
		Get_Robot_Pos_Frame = Robot_Pos_Frame;
	}

	// Store Force offset
	public void setForce_X_Offset(double Force_X_Offset) {
		Get_Force_X_Offset = Force_X_Offset;
	}

	// Store Force offset
	public void setForce_Y_Offset(double Force_Y_Offset) {
		Get_Force_Y_Offset = Force_Y_Offset;
	}

	// Store Force offset
	public void setForce_Z_Offset(double Force_Z_Offset) {
		Get_Force_Z_Offset = Force_Z_Offset;
	}

	// Store Force avg complete
	public void setForce_Avg_Comp(int Force_Avg_Comp) {
		Get_Force_Avg_Comp = Force_Avg_Comp;
	}

	/*********************************************************************************************
	 * GET Objects
	 *********************************************************************************************/
	@Override
	public Frame getRobotPosition() {
		return Get_Robot_Pos_Frame;
	}

	@Override
	public double getForce_X_Offset() {
		return Get_Force_X_Offset;
	}

	@Override
	public double getForce_Y_Offset() {
		return Get_Force_Y_Offset;
	}

	@Override
	public double getForce_Z_Offset() {
		return Get_Force_Z_Offset;
	}

	@Override
	public int getForce_Avg_Comp() {
		return Get_Force_Avg_Comp;
	}

	/*********************************************************************************************
	 * DEMO STUFF
	 *********************************************************************************************/
	// Demo stuff
	void setApplicationRecoveryInterface(IRecovery recovery) {
	}

	@Override
	public boolean isAssemblyRunning() {
		return _assembly;
	}

	public void setAssemblyRunning(boolean assembly) {
		_assembly = assembly;
	}

	@Override
	public boolean isManualRepositioningRequired() {

		return _applicationRecoveryInterface.isRecoveryRequired();
	}

	public void setApplicationRecoveryInterface1(IRecovery applicationRecoveryInterface) {
		_applicationRecoveryInterface = applicationRecoveryInterface;
	}

}
