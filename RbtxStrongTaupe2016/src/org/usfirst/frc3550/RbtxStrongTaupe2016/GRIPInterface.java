package org.usfirst.frc3550.RbtxStrongTaupe2016;

import java.io.IOException;

public class GRIPInterface {
	ProcessBuilder gripBase;
	Process gripRunner;
	GRIPInterface(String command){
		ProcessBuilder gripBase = new ProcessBuilder("/usr/local/frc/JRE/bin/java -jar -Xmx50m -XX:-OmitStackTraceInFastThrow -XX:+HeapDumpOnOutOfMemoryError"
				+ " grip.jar " + command);
		try {
			Process gripRunner = gripBase.start();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	boolean isGRIPRunning(boolean restart){// Retourne vrai si GRIP n'a pas crash et faux si il a crash. Si restart est positif, grip sera relance
		try{
			if(gripRunner.isAlive()){
				return true;
			}
			else if (restart){
				try {
					gripRunner = gripBase.start();
				} catch (NullPointerException e){
					e.printStackTrace();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				return false;
			}
			return false;
		}
		catch(NullPointerException e){
			e.printStackTrace();
		}
		return false;
	}
	void stopGRIP(){
		gripRunner.destroy();
	}
	void startGRIP(){
		try {
			gripRunner = gripBase.start();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
