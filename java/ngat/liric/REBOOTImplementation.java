// REBOOTImplementation.java
// $Id$
package ngat.liric;

import java.lang.*;
import java.io.IOException;
import java.util.*;

import ngat.message.base.*;
import ngat.message.ISS_INST.REBOOT;
import ngat.liric.command.*;
import ngat.util.*;
import ngat.util.logging.*;

/**
 * This class provides the implementation for the REBOOT command sent to a server using the
 * Java Message System.
 * @author Chris Mottram
 * @version $Revision$
 */
public class REBOOTImplementation extends CommandImplementation implements JMSCommandImplementation
{
	/**
	 * Revision Control System id string, showing the version of the Class.
	 */
	public final static String RCSID = new String("$Id$");
	/**
	 * Class constant used in calculating acknowledge times, when the acknowledge time connot be found in the
	 * configuration file.
	 */
	public final static int DEFAULT_ACKNOWLEDGE_TIME = 		300000;
	/**
	 * String representing the root part of the property key used to get the acknowledge time for 
	 * a certain level of reboot.
	 */
	public final static String ACK_TIME_PROPERTY_KEY_ROOT =	    "liric.reboot.acknowledge_time.";
	/**
	 * String representing the root part of the property key used to decide whether a certain level of reboot
	 * is enabled.
	 */
	public final static String ENABLE_PROPERTY_KEY_ROOT =       "liric.reboot.enable.";
	/**
	 * Set of constant strings representing levels of reboot. The levels currently start at 1, so index
	 * 0 is currently "NONE". These strings need to be kept in line with level constants defined in
	 * ngat.message.ISS_INST.REBOOT.
	 */
	public final static String REBOOT_LEVEL_LIST[] =  {"NONE","REDATUM","SOFTWARE","HARDWARE","POWER_OFF"};
	/**
	 * The hostname to send C layer commands to.
	 */
	protected String cLayerHostname = null;
	/**
	 * The port number, of the C layer, to send C layer commands to.
	 */
	protected int cLayerPortNumber = 0;

	/**
	 * Constructor.
	 */
	public REBOOTImplementation()
	{
		super();
	}

	/**
	 * This method allows us to determine which class of command this implementation class implements.
	 * This method returns &quot;ngat.message.ISS_INST.REBOOT&quot;.
	 * @return A string, the classname of the class of ngat.message command this class implements.
	 */
	public static String getImplementString()
	{
		return "ngat.message.ISS_INST.REBOOT";
	}

	/**
	 * This method gets the REBOOT command's acknowledge time. This time is dependant on the level.
	 * This is calculated as follows:
	 * <ul>
	 * <li>If the level is LEVEL_REDATUM, the number stored in &quot; 
	 * liric.reboot.acknowledge_time.REDATUM &quot; in the Liric properties file is the timeToComplete.
	 * <li>If the level is LEVEL_SOFTWARE, the number stored in &quot; 
	 * liric.reboot.acknowledge_time.SOFTWARE &quot; in the Liric properties file is the timeToComplete.
	 * <li>If the level is LEVEL_HARDWARE, the number stored in &quot; 
	 * liric.reboot.acknowledge_time.HARDWARE &quot; in the Liric properties file is the timeToComplete.
	 * <li>If the level is LEVEL_POWER_OFF, the number stored in &quot; 
	 * liric.reboot.acknowledge_time.POWER_OFF &quot; in the Liric properties file is the timeToComplete.
	 * </ul>
	 * If these numbers cannot be found, the default number DEFAULT_ACKNOWLEDGE_TIME is used instead.
	 * @param command The command instance we are implementing.
	 * @return An instance of ACK with the timeToComplete set to a time (in milliseconds).
	 * @see #DEFAULT_ACKNOWLEDGE_TIME
	 * @see #ACK_TIME_PROPERTY_KEY_ROOT
	 * @see #REBOOT_LEVEL_LIST
	 * @see ngat.message.base.ACK#setTimeToComplete
	 * @see LiricStatus#getPropertyInteger
	 */
	public ACK calculateAcknowledgeTime(COMMAND command)
	{
		ngat.message.ISS_INST.REBOOT rebootCommand = (ngat.message.ISS_INST.REBOOT)command;
		ACK acknowledge = null;
		int timeToComplete = 0;

		acknowledge = new ACK(command.getId()); 
		try
		{
			timeToComplete = status.getPropertyInteger(ACK_TIME_PROPERTY_KEY_ROOT+
								   REBOOT_LEVEL_LIST[rebootCommand.getLevel()]);
		}
		catch(Exception e)
		{
			liric.error(this.getClass().getName()+":calculateAcknowledgeTime:"+
				     rebootCommand.getLevel(),e);
			timeToComplete = DEFAULT_ACKNOWLEDGE_TIME;
		}
	//set time and return
		acknowledge.setTimeToComplete(timeToComplete);
		return acknowledge;
	}

	
	/**
	 * This method implements the REBOOT command. 
	 * An object of class REBOOT_DONE is returned.
	 * The <i>liric.reboot.enable.&lt;level&gt;</i> property is checked to see to whether to really
	 * do the specified level of reboot. This enables us to say, disbale to POWER_OFF reboot, if the
	 * instrument control computer is not connected to an addressable power supply.
	 * The following four levels of reboot are recognised:
	 * <ul>
	 * <li>REDATUM. This shuts down the connection to the controller, and then
	 * 	restarts it. 
	 * <li>SOFTWARE. This sends the "shutdown" command to the C layer, which stops 
	 *      (and via the autobooter, restarts) the C layer software. It then closes the
	 * 	server socket using the Liric close method. It then exits the Liric control software.
	 * <li>HARDWARE. This shuts down the connection to the Liric detector and closes the
	 * 	server socket using the Liric close method. It then issues a reboot
	 * 	command to the underlying operating system, to restart the instrument computer.
	 * <li>POWER_OFF. This closes the
	 * 	server socket using the Liric close method. It then issues a shutdown
	 * 	command to the underlying operating system, to put the instrument computer into a state
	 * 	where power can be switched off. 
	 * </ul>
	 * Note: You need to perform at least a SOFTWARE level reboot to re-read the Liric configuration file,
	 * as it contains information such as server ports.
	 * @param command The command instance we are implementing.
	 * @return An instance of REBOOT_DONE. Note this is only returned on a REDATUM level reboot,
	 *         all other levels cause the Moptop to terminate (either directly or indirectly) and a DONE
	 *         message cannot be returned.
	 * @see ngat.message.ISS_INST.REBOOT#LEVEL_REDATUM
	 * @see ngat.message.ISS_INST.REBOOT#LEVEL_SOFTWARE
	 * @see ngat.message.ISS_INST.REBOOT#LEVEL_HARDWARE
	 * @see ngat.message.ISS_INST.REBOOT#LEVEL_POWER_OFF
	 * @see #ENABLE_PROPERTY_KEY_ROOT
	 * @see #REBOOT_LEVEL_LIST
	 * @see #getCLayerConfig
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see #sendShutdownCommand
	 * @see Liric#close
	 */
	public COMMAND_DONE processCommand(COMMAND command)
	{
		ngat.message.ISS_INST.REBOOT rebootCommand = (ngat.message.ISS_INST.REBOOT)command;
		ngat.message.ISS_INST.REBOOT_DONE rebootDone = new ngat.message.ISS_INST.REBOOT_DONE(command.getId());
		ICSDRebootCommand icsdRebootCommand = null;
		ICSDShutdownCommand icsdShutdownCommand = null;
		LiricREBOOTQuitThread quitThread = null;
		boolean enable = false;

		liric.log(Logging.VERBOSITY_TERSE,this.getClass().getName()+":processCommand:Started.");
		try
		{
			// is reboot enabled at this level
			enable = status.getPropertyBoolean(ENABLE_PROPERTY_KEY_ROOT+
							   REBOOT_LEVEL_LIST[rebootCommand.getLevel()]);
			// if not enabled return OK
			if(enable == false)
			{
				liric.log(Logging.VERBOSITY_VERY_TERSE,"Command:"+
					   rebootCommand.getClass().getName()+":Level:"+rebootCommand.getLevel()+
					   " is not enabled.");
				rebootDone.setErrorNum(LiricConstants.LIRIC_ERROR_CODE_NO_ERROR);
				rebootDone.setErrorString("");
				rebootDone.setSuccessful(true);
				return rebootDone;
			}
			// get C layer comms configuration
			getCLayerConfig();
			// do relevent reboot based on level
			switch(rebootCommand.getLevel())
			{
				case REBOOT.LEVEL_REDATUM:
					liric.reInit();
					break;
				case REBOOT.LEVEL_SOFTWARE:
					// send software restart onto c layer.
					// send shutdown command to C Layer
					sendShutdownCommand(cLayerHostname,cLayerPortNumber);
					liric.close(serverConnectionThread);
					quitThread = new LiricREBOOTQuitThread("quit:"+rebootCommand.getId());
					quitThread.setLiric(liric);
					quitThread.setWaitThread(serverConnectionThread);
					// software will quit with exit value 0 as normal,
					// This will cause the autobooter to restart it.
					quitThread.start();
					break;
				case REBOOT.LEVEL_HARDWARE:
					liric.close(serverConnectionThread);
					quitThread = new LiricREBOOTQuitThread("quit:"+rebootCommand.getId());
					quitThread.setLiric(liric);
					quitThread.setWaitThread(serverConnectionThread);
					// tell the autobooter not to restart the control system
					quitThread.setExitValue(127);
					quitThread.start();
				// send reboot to the icsd_inet
					liric.log(Logging.VERBOSITY_TERSE,this.getClass().getName()+
						    ":processCommand:Sending reboot command to icsd_inet on machine "+
							   cLayerHostname+".");
					icsdRebootCommand = new ICSDRebootCommand(cLayerHostname);
					icsdRebootCommand.send();
					break;
				case REBOOT.LEVEL_POWER_OFF:
					liric.close(serverConnectionThread);
					quitThread = new LiricREBOOTQuitThread("quit:"+rebootCommand.getId());
					quitThread.setLiric(liric);
					quitThread.setWaitThread(serverConnectionThread);
					// tell the autobooter not to restart the control system
					quitThread.setExitValue(127);
					quitThread.start();
				// send shutdown to the icsd_inet
					liric.log(Logging.VERBOSITY_TERSE,this.getClass().getName()+
						    ":processCommand:Sending shutdown command to icsd_inet on machine "+
							   cLayerHostname+".");
					icsdShutdownCommand = new ICSDShutdownCommand(cLayerHostname);
					icsdShutdownCommand.send();
					break;
				default:
					liric.error(this.getClass().getName()+
						":processCommand:"+command+":Illegal level:"+rebootCommand.getLevel());
					rebootDone.setErrorNum(LiricConstants.LIRIC_ERROR_CODE_BASE+1400);
					rebootDone.setErrorString("Illegal level:"+rebootCommand.getLevel());
					rebootDone.setSuccessful(false);
					return rebootDone;
			};// end switch
		}
		catch(Exception e)
		{
			liric.error(this.getClass().getName()+
					":processCommand:"+command+":",e);
			rebootDone.setErrorNum(LiricConstants.LIRIC_ERROR_CODE_BASE+1404);
			rebootDone.setErrorString(e.toString());
			rebootDone.setSuccessful(false);
			return rebootDone;
		}
		liric.log(Logging.VERBOSITY_TERSE,this.getClass().getName()+":processCommand:Finished.");
	// return done object.
		rebootDone.setErrorNum(LiricConstants.LIRIC_ERROR_CODE_NO_ERROR);
		rebootDone.setErrorString("");
		rebootDone.setSuccessful(true);
		return rebootDone;
	}

	/**
	 * Get the C layer hostname and port number.
	 * @exception Exception Thrown if the relevant property can be received.
	 * @see #status
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see LiricStatus#getProperty
	 * @see LiricStatus#getPropertyInteger
	 */
	protected void getCLayerConfig() throws Exception
	{
		cLayerHostname = status.getProperty("liric.c.hostname");
		cLayerPortNumber = status.getPropertyInteger("liric.c.port_number");
	}

	/**
	 * Send a "shutdown" command to the C layer. An instance of ShutdownCommand is used to send the command
	 * to the C layer.
	 * @param hostname A string containing the hostname to send the shutdown command to.
	 * @param portNumber The port number of the C layer to send commands to.
	 * @exception Exception Thrown if an error occurs.
	 * @see ngat.liric.command.ShutdownCommand
	 */
	protected void sendShutdownCommand(String hostname,int portNumber) throws Exception
	{
		ShutdownCommand shutdownCommand = null;
		int returnCode,exposureStatus;
		String errorString = null;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"sendShutdownCommand("+hostname+":"+portNumber+"):started.");
		shutdownCommand = new ShutdownCommand();
		shutdownCommand.setAddress(hostname);
		shutdownCommand.setPortNumber(portNumber);
		// actually send the command to the C layer
		shutdownCommand.sendCommand();
		// check the parsed reply
		if(shutdownCommand.getParsedReplyOK() == false)
		{
			returnCode = shutdownCommand.getReturnCode();
			errorString = shutdownCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,"sendShutdownCommand:shutdown command for "+
				   hostname+":"+portNumber+" failed with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+":sendShutdownCommand:shutdown command for "+
					    hostname+":"+portNumber+" failed with return code "+
					    returnCode+" and error string:"+errorString);
		}
		liric.log(Logging.VERBOSITY_INTERMEDIATE,
			   "sendShutdownCommand("+hostname+":"+portNumber+"):finished.");
	}
}
