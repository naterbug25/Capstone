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

import com.kuka.roboticsAPI.geometricModel.Frame;

public interface IApplicationInformationFunction {
	/*********************************************************************************************
	 * GET Objects
	 *********************************************************************************************/
	public Frame getRobotPosition();

	public double getForce_X_Offset();

	public double getForce_Y_Offset();

	public double getForce_Z_Offset();

	public int getForce_Avg_Comp();

	/*********************************************************************************************
	 * DEMO STUFF
	 *********************************************************************************************/
	public boolean isAssemblyRunning();

	public boolean isManualRepositioningRequired();

}
