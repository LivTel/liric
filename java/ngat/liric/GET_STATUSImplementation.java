// GET_STATUSImplementation.java
// $Id$
package ngat.liric;

import java.lang.*;
import java.text.*;
import java.util.*;

import ngat.liric.command.*;
import ngat.message.base.*;
import ngat.message.base.*;
import ngat.message.ISS_INST.*;
import ngat.util.logging.*;
import ngat.util.ExecuteCommand;

/**
 * This class provides the implementation for the GET_STATUS command sent to a server using the
 * Java Message System.
 * @author Chris Mottram
 * @version $Revision$
 */
public class GET_STATUSImplementation extends CommandImplementation implements JMSCommandImplementation
{
	/**
	 * Revision Control System id string, showing the version of the Class.
	 */
	public final static String RCSID = new String("$Id$");
	/**
	 * This hashtable is created in processCommand, and filled with status data,
	 * and is returned in the GET_STATUS_DONE object.
	 * Could be declared:  Generic:&lt;String, Object&gt; but this is not supported by Java 1.4.
	 */
	private Hashtable hashTable = null;
	/**
	 * Standard status string passed back in the hashTable, describing the detector temperature status health,
	 * using the standard keyword KEYWORD_DETECTOR_TEMPERATURE_INSTRUMENT_STATUS. 
	 * Initialised to VALUE_STATUS_UNKNOWN.
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#KEYWORD_DETECTOR_TEMPERATURE_INSTRUMENT_STATUS
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#VALUE_STATUS_UNKNOWN
	 */
	private String detectorTemperatureInstrumentStatus = GET_STATUS_DONE.VALUE_STATUS_UNKNOWN;
	/**
	 * The current overall mode (status) of the RingoIII control system.
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#MODE_IDLE
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#MODE_EXPOSING
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#MODE_READING_OUT
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#MODE_ERROR
	 */
	protected int currentMode;
	/**
	 * The C layer's hostname.
	 */
	protected String cLayerHostname = null;
	/**
	 * The C layer's port number.
	 */	
	protected int cLayerPortNumber = 0;
	
	/**
	 * Constructor.
	 */
	public GET_STATUSImplementation()
	{
		super();
	}

	/**
	 * This method allows us to determine which class of command this implementation class implements.
	 * This method returns &quot;ngat.message.ISS_INST.GET_STATUS&quot;.
	 * @return A string, the classname of the class of ngat.message command this class implements.
	 */
	public static String getImplementString()
	{
		return "ngat.message.ISS_INST.GET_STATUS";
	}

	/**
	 * This method gets the GET_STATUS command's acknowledge time. 
	 * This takes the default acknowledge time to implement.
	 * @param command The command instance we are implementing.
	 * @return An instance of ACK with the timeToComplete set.
	 * @see ngat.message.base.ACK#setTimeToComplete
	 * @see LiricTCPServerConnectionThread#getDefaultAcknowledgeTime
	 */
	public ACK calculateAcknowledgeTime(COMMAND command)
	{
		ACK acknowledge = null;

		acknowledge = new ACK(command.getId());
		acknowledge.setTimeToComplete(serverConnectionThread.getDefaultAcknowledgeTime());
		return acknowledge;
	}

	/**
	 * This method implements the GET_STATUS command. 
	 * The local hashTable is setup (returned in the done object) and a local copy of status setup.
	 * <ul>
	 * <li>getCLayerConfig is called to get the C layer address/port number.
	 * <li>getExposureStatus is called to get the exposure status into the exposureStatus and exposureStatusString
	 *     variables.
	 * <li>"Exposure Status" and "Exposure Status String" status properties are added to the hashtable.
	 * <li>The "Instrument" status property is set to the "liric.get_status.instrument_name" property value.
	 * <li>The detectorTemperatureInstrumentStatus is initialised.
	 * <li>The "currentCommand" status hashtable value is set to the currently executing command.
	 * <li>getStatusExposureIndex / getStatusExposureCount / getStatusExposureMultrun / getStatusExposureRun / 
	 *     getStatusExposureWindow / getStatusExposureLength / 
	 *     getStatusExposureStartTime / getStatusExposureCoaddCount / getStatusExposureCoaddLength	
         *     are called to add some basic status to the hashtable.
	 * <li>getIntermediateStatus is called if the GET_STATUS command level is at least intermediate.
	 * <li>getFullStatusis called if the GET_STATUS command level is at least full.
	 * </ul>
	 * An object of class GET_STATUS_DONE is returned, with the information retrieved.
	 * @param command The GET_STATUS command.
	 * @return An object of class GET_STATUS_DONE is returned.
	 * @see #liric
	 * @see #status
	 * @see #hashTable
	 * @see #detectorTemperatureInstrumentStatus
	 * @see #getCLayerConfig
	 * @see #getExposureStatus
	 * @see #getStatusExposureIndex
	 * @see #getStatusExposureCount
	 * @see #getStatusExposureMultrun
	 * @see #getStatusExposureRun
	 * @see #getStatusExposureLength
	 * @see #getStatusExposureStartTime
	 * @see #getStatusExposureCoaddCount
	 * @see #getStatusExposureCoaddLength
	 * @see #getIntermediateStatus
	 * @see #getFullStatus
	 * @see #currentMode
	 * @see LiricStatus#getProperty
	 * @see LiricStatus#getCurrentCommand
	 * @see GET_STATUS#getLevel
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#KEYWORD_INSTRUMENT_STATUS
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#KEYWORD_DETECTOR_TEMPERATURE_INSTRUMENT_STATUS
	 */
	public COMMAND_DONE processCommand(COMMAND command)
	{
		GET_STATUS getStatusCommand = (GET_STATUS)command;
		GET_STATUS_DONE getStatusDone = new GET_STATUS_DONE(command.getId());
		ISS_TO_INST currentCommand = null;

		try
		{
			// Create new hashtable to be returned
			// v1.5 generic typing of collections:<String, Object>, can't be used due to v1.4 compatibility
			hashTable = new Hashtable();
			// get C layer comms configuration
			getCLayerConfig();
			// exposure status
			// Also sets currentMode
			getExposureStatus(); 
			getStatusDone.setCurrentMode(currentMode); 
			// What instrument is this?
			hashTable.put("Instrument",status.getProperty("liric.get_status.instrument_name"));
			// Initialise Standard status to UNKNOWN
			detectorTemperatureInstrumentStatus = GET_STATUS_DONE.VALUE_STATUS_UNKNOWN;
			hashTable.put(GET_STATUS_DONE.KEYWORD_DETECTOR_TEMPERATURE_INSTRUMENT_STATUS,
				      detectorTemperatureInstrumentStatus);
			hashTable.put(GET_STATUS_DONE.KEYWORD_INSTRUMENT_STATUS,GET_STATUS_DONE.VALUE_STATUS_UNKNOWN);
			// current command
			currentCommand = status.getCurrentCommand();
			if(currentCommand == null)
				hashTable.put("currentCommand","");
			else
				hashTable.put("currentCommand",currentCommand.getClass().getName());
			// basic information
			getFilterWheelStatus();
			// "Exposure Count" is searched for by the IcsGUI
			getStatusExposureCount(); 
			// "Exposure Length" is needed for IcsGUI
			getStatusExposureLength();  
			// "Exposure Start Time" is needed for IcsGUI
			getStatusExposureStartTime(); 
			// "Elapsed Exposure Time" is needed for IcsGUI.
			// This requires "Exposure Length" and "Exposure Start Time hashtable entries to have
			// already been inserted by getStatusExposureLength/getStatusExposureStartTime
			getStatusExposureElapsedTime();
			// "Exposure Number" is added in getStatusExposureIndex
			getStatusExposureIndex();
			// Coadd exposure length and count
			getStatusExposureCoaddCount();
			getStatusExposureCoaddLength();
			getStatusExposureMultrun(); 
			getStatusExposureRun();
		}
		catch(Exception e)
		{
			liric.error(this.getClass().getName()+
				       ":processCommand:Retrieving basic status failed.",e);
			getStatusDone.setDisplayInfo(hashTable);
			getStatusDone.setErrorNum(LiricConstants.LIRIC_ERROR_CODE_BASE+2500);
			getStatusDone.setErrorString("processCommand:Retrieving basic status failed:"+e);
			getStatusDone.setSuccessful(false);
			return getStatusDone;
		}
	// intermediate level information - basic plus controller calls.
		if(getStatusCommand.getLevel() >= GET_STATUS.LEVEL_INTERMEDIATE)
		{
			getIntermediateStatus();
		}// end if intermediate level status
	// Get full status information.
		if(getStatusCommand.getLevel() >= GET_STATUS.LEVEL_FULL)
		{
			getFullStatus();
		}
	// set hashtable and return values.
		getStatusDone.setDisplayInfo(hashTable);
		getStatusDone.setErrorNum(LiricConstants.LIRIC_ERROR_CODE_NO_ERROR);
		getStatusDone.setErrorString("");
		getStatusDone.setSuccessful(true);
	// return done object.
		return getStatusDone;
	}

	/**
	 * Get the C layer's hostname and port number from the properties into some internal variables. 
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
	 * Get the status/position of the filter wheel.
	 * @exception Exception Thrown if an error occurs.
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see ngat.liric.command.StatusFilterWheelFilterCommand
	 * @see ngat.liric.command.StatusFilterWheelPositionCommand
	 * @see ngat.liric.command.StatusFilterWheelStatusCommand
	 */
	protected void getFilterWheelStatus() throws Exception
	{
		StatusFilterWheelFilterCommand statusFilterWheelFilterCommand = null;
		StatusFilterWheelPositionCommand statusFilterWheelPositionCommand = null;
		StatusFilterWheelStatusCommand statusFilterWheelStatusCommand = null;
		String errorString = null;
		String filterName = null;
		String filterWheelStatus = null;
		int returnCode,filterWheelPosition;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getFilterWheelStatus:started for C layer :Hostname: "+
			   cLayerHostname+" Port Number: "+cLayerPortNumber+".");
		// Setup StatusFilterWheelFilterCommand
		statusFilterWheelFilterCommand = new StatusFilterWheelFilterCommand();
		statusFilterWheelFilterCommand.setAddress(cLayerHostname);
		statusFilterWheelFilterCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusFilterWheelFilterCommand.sendCommand();
		// check the parsed reply
		if(statusFilterWheelFilterCommand.getParsedReplyOK() == false)
		{
			returnCode = statusFilterWheelFilterCommand.getReturnCode();
			errorString = statusFilterWheelFilterCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getFilterWheelStatus:filterwheel filter command failed with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+
					  ":getFilterWheelStatus:filterwheel filter command failed with return code "+
					    returnCode+" and error string:"+errorString);
		}
		filterName = statusFilterWheelFilterCommand.getFilterName();
		hashTable.put("Filter Wheel:1",new String(filterName));
		// "status filterwheel position" command
		statusFilterWheelPositionCommand = new StatusFilterWheelPositionCommand();
		statusFilterWheelPositionCommand.setAddress(cLayerHostname);
		statusFilterWheelPositionCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusFilterWheelPositionCommand.sendCommand();
		// check the parsed reply
		if(statusFilterWheelPositionCommand.getParsedReplyOK() == false)
		{
			returnCode = statusFilterWheelPositionCommand.getReturnCode();
			errorString = statusFilterWheelPositionCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getFilterWheelStatus:filterwheel position command failed with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+
					":getFilterWheelStatus:filterwheel position command failed with return code "+
					    returnCode+" and error string:"+errorString);
		}
		filterWheelPosition = statusFilterWheelPositionCommand.getFilterWheelPosition();
		hashTable.put("Filter Wheel Position:1",new Integer(filterWheelPosition));
		// "status filterwheel status" command
		statusFilterWheelStatusCommand = new StatusFilterWheelStatusCommand();
		statusFilterWheelStatusCommand.setAddress(cLayerHostname);
		statusFilterWheelStatusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusFilterWheelStatusCommand.sendCommand();
		// check the parsed reply
		if(statusFilterWheelStatusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusFilterWheelStatusCommand.getReturnCode();
			errorString = statusFilterWheelStatusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getFilterWheelStatus:filterwheel status command failed with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+
					   ":getFilterWheelStatus:filterwheel status command failed with return code "+
					    returnCode+" and error string:"+errorString);
		}
		filterWheelStatus = statusFilterWheelStatusCommand.getFilterWheelStatus();
		hashTable.put("Filter Wheel Status:1",new String(filterWheelStatus));
	}
	
	/**
	 * Get the exposure status. 
	 * This retrieved using an instance of StatusExposureStatusCommand.
	 * The "Multrun In Progress" keyword/value pairs are generated from the returned status. 
	 * The currentMode is set as either MODE_IDLE, or MODE_EXPOSING if the C layer reports a multrun is in progress.
	 * @exception Exception Thrown if an error occurs.
	 * @see #currentMode
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see ngat.liric.command.StatusExposureStatusCommand
	 * @see ngat.liric.command.StatusExposureStatusCommand#getMultrunInProgress
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#MODE_IDLE
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#MODE_EXPOSING
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#MODE_ERROR
	 */
	protected void getExposureStatus() throws Exception
	{
		StatusExposureStatusCommand statusCommand = null;
		int returnCode;
		String errorString = null;
		boolean multrunInProgress;
		
		// initialise currentMode to IDLE
		currentMode = GET_STATUS_DONE.MODE_IDLE;
		// Setup StatusExposureStatusCommand
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getExposureStatus:started for C layer :Hostname: "+
			   cLayerHostname+" Port Number: "+cLayerPortNumber+".");
		statusCommand = new StatusExposureStatusCommand();
		statusCommand.setAddress(cLayerHostname);
		statusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusCommand.sendCommand();
		// check the parsed reply
		if(statusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusCommand.getReturnCode();
			errorString = statusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getExposureStatus:exposure status command failed with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+
					    ":getExposureStatus:exposure status command failed with return code "+
					    returnCode+" and error string:"+errorString);
		}
		multrunInProgress = statusCommand.getMultrunInProgress();
		hashTable.put("Multrun In Progress",new Boolean(multrunInProgress));
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getExposureStatus:finished with multrun in progress:"+
			   multrunInProgress);
		// change currentMode dependant on what the C layer is doing
		// We can only detect whether a multrun is in progress in Liric
		if(multrunInProgress)
			currentMode = GET_STATUS_DONE.MODE_EXPOSING;
	}

	/**
	 * Get the exposure count. An instance of StatusExposureCountCommand is used to send the command
	 * to the C layer, The returned value is stored in
	 * the hashTable, under the "Exposure Count" key. 
	 * @exception Exception Thrown if an error occurs.
	 * @see #hashTable
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see ngat.liric.command.StatusExposureCountCommand
	 */
	protected void getStatusExposureCount() throws Exception
	{
		StatusExposureCountCommand statusCommand = null;
		int returnCode,exposureCount;
		String errorString = null;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureCount:started for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+").");
		statusCommand = new StatusExposureCountCommand();
		statusCommand.setAddress(cLayerHostname);
		statusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusCommand.sendCommand();
		// check the parsed reply
		if(statusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusCommand.getReturnCode();
			errorString = statusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getStatusExposureCount:exposure count command failed ("+
				   cLayerHostname+":"+cLayerPortNumber+") with return code "+
				   returnCode+" and error string:"+errorString);
				throw new Exception(this.getClass().getName()+
						    ":getStatusExposureCount:"+
						    "exposure count command failed ("+cLayerHostname+":"+
						    cLayerPortNumber+") with return code "+returnCode+
						    " and error string:"+errorString);
		}
		exposureCount = statusCommand.getExposureCount();
		hashTable.put("Exposure Count",new Integer(exposureCount));
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureCount:finished for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+") with count:"+exposureCount);
	}

	/**
	 * Get the exposure length. An instance of StatusExposureLengthCommand is used to send the command
	 * to the C layer. The returned value is stored in
	 * the hashTable, under the "Exposure Length" key.
	 * @exception Exception Thrown if an error occurs.
	 * @see #hashTable
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see ngat.liric.command.StatusExposureLengthCommand
	 */
	protected void getStatusExposureLength() throws Exception
	{
		StatusExposureLengthCommand statusCommand = null;
		int returnCode,exposureLength;
		String errorString = null;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureLength:started for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+").");
		statusCommand = new StatusExposureLengthCommand();
		statusCommand.setAddress(cLayerHostname);
		statusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusCommand.sendCommand();
		// check the parsed reply
		if(statusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusCommand.getReturnCode();
			errorString = statusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getStatusExposureLength:exposure length command failed for C layer ("+
				   cLayerHostname+":"+cLayerPortNumber+") with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+":getStatusExposureLength:"+
					    "exposure length command failed for C layer ("+
					    cLayerHostname+":"+cLayerPortNumber+
					    ") with return code "+returnCode+" and error string:"+errorString);
		}
		exposureLength = statusCommand.getExposureLength();
		hashTable.put("Exposure Length",new Integer(exposureLength));
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureLength:finished for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+") with length:"+exposureLength+ " ms.");
	}
	
	/**
	 * Get the exposure start time. An instance of StatusExposureStartTimeCommand is used to send the command
	 * to the C layer. The returned value is stored in
	 * the hashTable, under the "Exposure Start Time" and "Exposure Start Time Date" key.
	 * @exception Exception Thrown if an error occurs.
	 * @see #hashTable
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see ngat.liric.command.StatusExposureStartTimeCommand
	 */
	protected void getStatusExposureStartTime() throws Exception
	{
		StatusExposureStartTimeCommand statusCommand = null;
		Date exposureStartTime = null;
		int returnCode;
		String errorString = null;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureStartTime:started.");
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureStartTime:started for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+").");
		statusCommand = new StatusExposureStartTimeCommand();
		statusCommand.setAddress(cLayerHostname);
		statusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusCommand.sendCommand();
		// check the parsed reply
		if(statusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusCommand.getReturnCode();
			errorString = statusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,"getStatusExposureStartTime:"+
				   "exposure start time command failed for C layer ("+
				   cLayerHostname+":"+cLayerPortNumber+") with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+":getStatusExposureStartTime:"+
					    "exposure start time command failed for C layer ("+
					    cLayerHostname+":"+cLayerPortNumber+") with return code "+
					    returnCode+" and error string:"+errorString);
		}
		exposureStartTime = statusCommand.getTimestamp();
		hashTable.put("Exposure Start Time",new Long(exposureStartTime.getTime()));
		hashTable.put("Exposure Start Time Date",exposureStartTime);
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureStartTime:finished for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+") with start time:"+exposureStartTime);
	}

	/**
	 * Compute and insert the "Elapsed Exposure Time" key. 
	 * The IcsgUI calculates the remaining exposure time by doing the 
	 * calculation(exposureLength-elapsedExposureTime).
	 * Here, we do the following:
	 * <ul>
	 * <li>We get the current time nowTime.
	 * <li>We retrieve the "Exposure Start Time" hashtable value 
	 *     (previously generated in getStatusExposureStartTime).
	 * <li>We retrieve the "Exposure Length" hashtable value 
	 *     (previously generated in getStatusExposureLength).
	 * <li>We compute the elapsed exposure time as being nowTime minus the exposure start time.
	 * <li>We set the "Elapsed Exposure Time" hashTable key to the computed elapsed exposure time.
	 * </ul>
	 * @see #hashTable
	 * @see #getStatusExposureStartTime
	 * @see #getStatusExposureLength
	 */
	protected void getStatusExposureElapsedTime()
	{
		Date nowTime = null;
		Integer exposureLengthObject = null;
		Long exposureStartTimeObject = null;
		long exposureStartTime;
	        int exposureLength;
	        int elapsedExposureTime;
		
		nowTime = new Date();
		exposureStartTimeObject = (Long)(hashTable.get("Exposure Start Time"));
		exposureLengthObject = (Integer)(hashTable.get("Exposure Length"));
		exposureStartTime = exposureStartTimeObject.longValue();
		exposureLength = exposureLengthObject.intValue();
		elapsedExposureTime = (int)(nowTime.getTime()-exposureStartTime);
		hashTable.put("Elapsed Exposure Time",new Integer(elapsedExposureTime));
	}
	
	/**
	 * Get the exposure index for each camera. 
	 * An instance of StatusExposureIndexCommand is used to send the command to the C layer. 
	 * The returned values are stored in the hashTable, under the : "Exposure Index" key.
	 * @exception Exception Thrown if an error occurs.
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see #hashTable
	 * @see ngat.liric.command.StatusExposureIndexCommand
	 */
	protected void getStatusExposureIndex() throws Exception
	{
		StatusExposureIndexCommand statusCommand = null;
		int returnCode,exposureIndex;
		String errorString = null;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureIndex:started for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+").");
		statusCommand = new StatusExposureIndexCommand();
		statusCommand.setAddress(cLayerHostname);
		statusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusCommand.sendCommand();
		// check the parsed reply
		if(statusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusCommand.getReturnCode();
			errorString = statusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getStatusExposureIndex:exposure index command failed for C layer ("+
				   cLayerHostname+":"+cLayerPortNumber+") with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+
					    ":getStatusExposureIndex:exposure index command failed for C layer ("+
					    cLayerHostname+":"+cLayerPortNumber+
					    ") with return code "+returnCode+" and error string:"+errorString);
		}
		exposureIndex = statusCommand.getExposureIndex();
		hashTable.put("Exposure Index",new Integer(exposureIndex));
		// exposure number is really the same thing, but is used by the IcsGUI.
		hashTable.put("Exposure Number",new Integer(exposureIndex));
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureIndex:finished for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+") with index:"+exposureIndex);
	}
	
	/**
	 * Get the coadd exposure count. An instance of StatusExposureCoaddCountCommand is used to send the command
	 * to the C layer, The returned value is stored in
	 * the hashTable, under the "Coadd Exposure Count" key. 
	 * @exception Exception Thrown if an error occurs.
	 * @see #hashTable
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see ngat.liric.command.StatusExposureCountCommand
	 */
	protected void getStatusExposureCoaddCount() throws Exception
	{
		StatusExposureCoaddCountCommand statusCommand = null;
		int returnCode,coaddExposureCount;
		String errorString = null;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureCoaddCount:started for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+").");
		statusCommand = new StatusExposureCoaddCountCommand();
		statusCommand.setAddress(cLayerHostname);
		statusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusCommand.sendCommand();
		// check the parsed reply
		if(statusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusCommand.getReturnCode();
			errorString = statusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getStatusExposureCoaddCount:exposure coadd count command failed ("+
				   cLayerHostname+":"+cLayerPortNumber+") with return code "+
				   returnCode+" and error string:"+errorString);
				throw new Exception(this.getClass().getName()+
						    ":getStatusExposureCoaddCount:"+
						    "exposure coadd count command failed ("+cLayerHostname+":"+
						    cLayerPortNumber+") with return code "+returnCode+
						    " and error string:"+errorString);
		}
		coaddExposureCount = statusCommand.getExposureCoaddCount();
		hashTable.put("Coadd Exposure Count",new Integer(coaddExposureCount));
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureCoaddCount:finished for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+") with coadd exposure count:"+coaddExposureCount);
	}

	/**
	 * Get the exposure coadd length. An instance of StatusExposureCoaddLengthCommand is used to send the command
	 * to the C layer. The returned value is stored in
	 * the hashTable, under the "Coadd Exposure Length" key.
	 * @exception Exception Thrown if an error occurs.
	 * @see #hashTable
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see ngat.liric.command.StatusExposureCoaddLengthCommand
	 */
	protected void getStatusExposureCoaddLength() throws Exception
	{
		StatusExposureCoaddLengthCommand statusCommand = null;
		int returnCode,coaddExposureLength;
		String errorString = null;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureCoaddLength:started for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+").");
		statusCommand = new StatusExposureCoaddLengthCommand();
		statusCommand.setAddress(cLayerHostname);
		statusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusCommand.sendCommand();
		// check the parsed reply
		if(statusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusCommand.getReturnCode();
			errorString = statusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getStatusExposureCoaddLength:exposure coadd length command failed for C layer ("+
				   cLayerHostname+":"+cLayerPortNumber+") with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+":getStatusExposureCoaddLength:"+
					    "exposure coadd length command failed for C layer ("+
					    cLayerHostname+":"+cLayerPortNumber+
					    ") with return code "+returnCode+" and error string:"+errorString);
		}
		coaddExposureLength = statusCommand.getExposureCoaddLength();
		hashTable.put("Coadd Exposure Length",new Integer(coaddExposureLength));
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureCoaddLength:finished for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+") with coadd exposure length:"+coaddExposureLength+ " ms.");
	}
	
	/**
	 * Get the exposure multrun. An instance of StatusExposureMultrunCommand is used to send the command
	 * to the C layer. The returned value is stored inthe hashTable, under the "Exposure Multrun" key.
	 * @exception Exception Thrown if an error occurs.
	 * @see #hashTable
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see ngat.liric.command.StatusExposureMultrunCommand
	 */
	protected void getStatusExposureMultrun() throws Exception
	{
		StatusExposureMultrunCommand statusCommand = null;
		int returnCode,exposureMultrun;
		String errorString = null;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureMultrun:started for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+").");
		statusCommand = new StatusExposureMultrunCommand();
		statusCommand.setAddress(cLayerHostname);
		statusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusCommand.sendCommand();
		// check the parsed reply
		if(statusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusCommand.getReturnCode();
			errorString = statusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getStatusExposureMultrun:exposure multrun command failed for C layer ("+
				   cLayerHostname+":"+cLayerPortNumber+") with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+":getStatusExposureMultrun:"+
					    "exposure multrun command failed for C layer ("+
					    cLayerHostname+":"+cLayerPortNumber+
					    ") with return code "+returnCode+" and error string:"+errorString);
		}
		exposureMultrun = statusCommand.getExposureMultrun();
		hashTable.put("Exposure Multrun",new Integer(exposureMultrun));
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureMultrun:finished for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+") with multrun number:"+exposureMultrun);
	}

	/**
	 * Get the exposure multrun run. An instance of StatusExposureRunCommand is used to send the command
	 * to the C layer. The returned value is stored in the hashTable, under the "Exposure Run" key.
	 * @exception Exception Thrown if an error occurs.
	 * @see #hashTable
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see ngat.liric.command.StatusExposureRunCommand
	 */
	protected void getStatusExposureRun() throws Exception
	{
		StatusExposureRunCommand statusCommand = null;
		int returnCode,exposureRun;
		String errorString = null;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureRun:started.");
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureRun:started for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+").");
		statusCommand = new StatusExposureRunCommand();
		statusCommand.setAddress(cLayerHostname);
		statusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusCommand.sendCommand();
		// check the parsed reply
		if(statusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusCommand.getReturnCode();
			errorString = statusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getStatusExposureRun:exposure run command failed for C layer ("+
				   cLayerHostname+":"+cLayerPortNumber+") with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+":getStatusExposureRun:"+
					    "exposure run command failed for C layer ("+
					    cLayerHostname+":"+cLayerPortNumber+
					    ") with return code "+returnCode+" and error string:"+errorString);
		}
		exposureRun = statusCommand.getExposureRun();
		hashTable.put("Exposure Run",new Integer(exposureRun));
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getStatusExposureRun:finished for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+") with run number:"+exposureRun);
	}
	
	/**
	 * Get intermediate level status. This is:
	 * <ul>
	 * <li>detctor temperature information from the camera.
	 * <li>Nudgematic status.
	 * </ul>
	 * The overall health and well-being statii are then computed using setInstrumentStatus.
	 * @see #getTemperature
	 * @see #getNudgematicStatus
	 * @see #setInstrumentStatus
	 */
	private void getIntermediateStatus()
	{
		try
		{
			getTemperature();
		}
		catch(Exception e)
		{
			liric.error(this.getClass().getName()+
				     ":getIntermediateStatus:Retrieving temperature status failed.",e);
		}
		try
		{
			getNudgematicStatus();
		}
		catch(Exception e)
		{
			liric.error(this.getClass().getName()+
				     ":getIntermediateStatus:Retrieving nudgematic status failed.",e);
		}
	// Standard status
		setInstrumentStatus();
	}

	/**
	 * Get the current detector temperature.
	 * An instance of StatusTemperatureGetCommand is used to send the command
	 * to the C layer. The returned value is stored in
	 * the hashTable, under the "Temperature" key (converted to Kelvin). 
	 * A timestamp is also retrieved (when the temperature was actually measured, it may be a cached value), 
	 * and this is stored in the "Temperature Timestamp" key.
	 * setDetectorTemperatureInstrumentStatus is called with the detector temperature to set
	 * the detector temperature health and wellbeing values.
	 * @exception Exception Thrown if an error occurs.
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see #hashTable
	 * @see #setDetectorTemperatureInstrumentStatus
	 * @see ngat.liric.Liric#CENTIGRADE_TO_KELVIN
	 * @see ngat.liric.command.StatusTemperatureGetCommand
	 * @see ngat.liric.command.StatusTemperatureGetCommand#setAddress
	 * @see ngat.liric.command.StatusTemperatureGetCommand#setPortNumber
	 * @see ngat.liric.command.StatusTemperatureGetCommand#sendCommand
	 * @see ngat.liric.command.StatusTemperatureGetCommand#getReturnCode
	 * @see ngat.liric.command.StatusTemperatureGetCommand#getParsedReply
	 * @see ngat.liric.command.StatusTemperatureGetCommand#getTemperature
	 * @see ngat.liric.command.StatusTemperatureGetCommand#getTimestamp
	 */
	protected void getTemperature() throws Exception
	{
		StatusTemperatureGetCommand statusCommand = null;
		int returnCode;
		String errorString = null;
		double temperature;
		Date timestamp;

		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getTemperature:started for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+").");
		statusCommand = new StatusTemperatureGetCommand();
		statusCommand.setAddress(cLayerHostname);
		statusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusCommand.sendCommand();
		// check the parsed reply
		if(statusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusCommand.getReturnCode();
			errorString = statusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getTemperature:status temperature get command failed for C layer ("+
				   cLayerHostname+":"+cLayerPortNumber+
				   ") with return code "+returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+":getTemperature:"+
					    "status temperature get command failed C layer ("+
					    cLayerHostname+":"+cLayerPortNumber+
					    ") with return code "+returnCode+" and error string:"+errorString);
		}
		temperature = statusCommand.getTemperature();
		timestamp = statusCommand.getTimestamp();
		hashTable.put("Temperature",new Double(temperature+Liric.CENTIGRADE_TO_KELVIN));
		hashTable.put("Temperature Timestamp",timestamp);
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getTemperature:finished for C layer ("+
			   cLayerHostname+":"+cLayerPortNumber+") with temperature:"+temperature+
			   " measured at "+timestamp);
		setDetectorTemperatureInstrumentStatus(temperature);
		liric.log(Logging.VERBOSITY_INTERMEDIATE,"getTemperature:finished.");
	}

	/**
	 * Set the standard entry for detector temperature in the hashtable based upon the current temperature.
	 * Reads the folowing config:
	 * <ul>
	 * <li>liric.get_status.detector.temperature.warm.warn
	 * <li>liric.get_status.detector.temperature.warm.fail
	 * <li>liric.get_status.detector.temperature.cold.warn
	 * <li>liric.get_status.detector.temperature.cold.fail
	 * </ul>
	 * @param temperature The current detector temperature in degrees C.
	 * @exception NumberFormatException Thrown if the config is not a valid double.
	 * @see #hashTable
	 * @see #status
	 * @see #detectorTemperatureInstrumentStatus
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#KEYWORD_DETECTOR_TEMPERATURE_INSTRUMENT_STATUS
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#VALUE_STATUS_OK
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#VALUE_STATUS_WARN
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#VALUE_STATUS_FAIL
	 */
	protected void setDetectorTemperatureInstrumentStatus(double temperature) throws NumberFormatException
	{
		double warmWarnTemperature,warmFailTemperature,coldWarnTemperature,coldFailTemperature;

		// get config for warn and fail temperatures
		warmWarnTemperature = status.getPropertyDouble("liric.get_status.detector.temperature.warm.warn");
		warmFailTemperature = status.getPropertyDouble("liric.get_status.detector.temperature.warm.fail");
		coldWarnTemperature = status.getPropertyDouble("liric.get_status.detector.temperature.cold.warn");
		coldFailTemperature = status.getPropertyDouble("liric.get_status.detector.temperature.cold.fail");
		// initialise status to OK
		detectorTemperatureInstrumentStatus = GET_STATUS_DONE.VALUE_STATUS_OK;
		if(temperature > warmFailTemperature)
		{
			detectorTemperatureInstrumentStatus = GET_STATUS_DONE.VALUE_STATUS_FAIL;
		}
		else if(temperature > warmWarnTemperature)
		{
			// only set to WARN if we are currently OKAY (i.e. if we are FAIL stay FAIL) 
			if(detectorTemperatureInstrumentStatus == GET_STATUS_DONE.VALUE_STATUS_OK)
			{
				detectorTemperatureInstrumentStatus = GET_STATUS_DONE.VALUE_STATUS_WARN;
			}
		}
		else if(temperature < coldFailTemperature)
		{
			detectorTemperatureInstrumentStatus = GET_STATUS_DONE.VALUE_STATUS_FAIL;
		}
		else if(temperature < coldWarnTemperature)
		{
			if(detectorTemperatureInstrumentStatus == GET_STATUS_DONE.VALUE_STATUS_OK)
			{
				// only set to WARN if we are currently OKAY (i.e. if we are FAIL stay FAIL) 
				detectorTemperatureInstrumentStatus = GET_STATUS_DONE.VALUE_STATUS_WARN;
			}
		}
		// set hashtable entry
		hashTable.put(GET_STATUS_DONE.KEYWORD_DETECTOR_TEMPERATURE_INSTRUMENT_STATUS,
			      detectorTemperatureInstrumentStatus);
	}

	/**
	 * Set the overall instrument status keyword in the hashtable. This is derived from sub-system keyword values,
	 * currently only the detector temperature. HashTable entry KEYWORD_INSTRUMENT_STATUS)
	 * should be set to the worst of OK/WARN/FAIL. If sub-systems are UNKNOWN, OK is returned.
	 * @see #hashTable
	 * @see #status
	 * @see #detectorTemperatureInstrumentStatus
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#KEYWORD_INSTRUMENT_STATUS
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#VALUE_STATUS_OK
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#VALUE_STATUS_WARN
	 * @see ngat.message.ISS_INST.GET_STATUS_DONE#VALUE_STATUS_FAIL
	 */
	protected void setInstrumentStatus()
	{
		String instrumentStatus;

		// default to OK
		instrumentStatus = GET_STATUS_DONE.VALUE_STATUS_OK;
		// if a sub-status is in warning, overall status is in warning
		if(detectorTemperatureInstrumentStatus.equals(GET_STATUS_DONE.VALUE_STATUS_WARN))
			instrumentStatus = GET_STATUS_DONE.VALUE_STATUS_WARN;
		// if a sub-status is in fail, overall status is in fail. This overrides a previous warn
	        if(detectorTemperatureInstrumentStatus.equals(GET_STATUS_DONE.VALUE_STATUS_FAIL))
			instrumentStatus = GET_STATUS_DONE.VALUE_STATUS_FAIL;
		// set standard status in hashtable
		hashTable.put(GET_STATUS_DONE.KEYWORD_INSTRUMENT_STATUS,instrumentStatus);
	}

	/**
	 * Get the status/position/offsetsize of the nudgematic mechanism.
	 * @exception Exception Thrown if an error occurs.
	 * @see #cLayerHostname
	 * @see #cLayerPortNumber
	 * @see ngat.liric.command.StatusNudgematicPositionCommand
	 * @see ngat.liric.command.StatusNudgematicStatusCommand
	 * @see ngat.liric.command.StatusNudgematicOffsetSizeCommand
	 */
	protected void getNudgematicStatus() throws Exception
	{
		StatusNudgematicPositionCommand statusNudgematicPositionCommand = null;
		StatusNudgematicStatusCommand statusNudgematicStatusCommand = null;
		StatusNudgematicOffsetSizeCommand statusNudgematicOffsetSizeCommand = null;
		
		String errorString = null;
		String nudgematicStatus = null;
		String nudgematicOffsetSize = null;
		int returnCode;
		int nudgematicPosition;

		// "status nudgematic position" command
		statusNudgematicPositionCommand = new StatusNudgematicPositionCommand();
		statusNudgematicPositionCommand.setAddress(cLayerHostname);
		statusNudgematicPositionCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusNudgematicPositionCommand.sendCommand();
		// check the parsed reply
		if(statusNudgematicPositionCommand.getParsedReplyOK() == false)
		{
			returnCode = statusNudgematicPositionCommand.getReturnCode();
			errorString = statusNudgematicPositionCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getNudgematicStatus:nudgematic position command failed with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+
					   ":getNudgematicStatus:nudgematic position command failed with return code "+
					    returnCode+" and error string:"+errorString);
		}
		nudgematicPosition = statusNudgematicPositionCommand.getNudgematicPosition();
		hashTable.put("Nudgematic Position",new Integer(nudgematicPosition));
		// "status nudgematic status" command
		statusNudgematicStatusCommand = new StatusNudgematicStatusCommand();
		statusNudgematicStatusCommand.setAddress(cLayerHostname);
		statusNudgematicStatusCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusNudgematicStatusCommand.sendCommand();
		// check the parsed reply
		if(statusNudgematicStatusCommand.getParsedReplyOK() == false)
		{
			returnCode = statusNudgematicStatusCommand.getReturnCode();
			errorString = statusNudgematicStatusCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getNudgematicStatus:nudgematic status command failed with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+
					    ":getNudgematicStatus:nudgematic status command failed with return code "+
					    returnCode+" and error string:"+errorString);
		}
		nudgematicStatus = statusNudgematicStatusCommand.getNudgematicStatus();
		hashTable.put("Nudgematic Status",new String(nudgematicStatus));
		// "status nudgematic offset size" command
		statusNudgematicOffsetSizeCommand = new StatusNudgematicOffsetSizeCommand();
		statusNudgematicOffsetSizeCommand.setAddress(cLayerHostname);
		statusNudgematicOffsetSizeCommand.setPortNumber(cLayerPortNumber);
		// actually send the command to the C layer
		statusNudgematicOffsetSizeCommand.sendCommand();
		// check the parsed reply
		if(statusNudgematicOffsetSizeCommand.getParsedReplyOK() == false)
		{
			returnCode = statusNudgematicOffsetSizeCommand.getReturnCode();
			errorString = statusNudgematicOffsetSizeCommand.getParsedReply();
			liric.log(Logging.VERBOSITY_TERSE,
				   "getNudgematicStatus:nudgematic status command failed with return code "+
				   returnCode+" and error string:"+errorString);
			throw new Exception(this.getClass().getName()+
					    ":getNudgematicStatus:nudgematic status command failed with return code "+
					    returnCode+" and error string:"+errorString);
		}
		nudgematicOffsetSize = statusNudgematicOffsetSizeCommand.getNudgematicOffsetSize();
		hashTable.put("Nudgematic Offset Size",new String(nudgematicOffsetSize));
	}

	/**
	 * Method to get misc status, when level FULL has been selected.
	 * The following data is put into the hashTable:
	 * <ul>
	 * <li><b>Log Level</b> The current logging level Liric is using.
	 * <li><b>Disk Usage</b> The results of running a &quot;df -k&quot;, to get the disk usage.
	 * <li><b>Process List</b> The results of running a &quot;ps -e -o pid,pcpu,vsz,ruser,stime,time,args&quot;, 
	 * 	to get the processes running on this machine.
	 * <li><b>Uptime</b> The results of running a &quot;uptime&quot;, 
	 * 	to get system load and time since last reboot.
	 * <li><b>Total Memory, Free Memory</b> The total and free memory in the Java virtual machine.
	 * <li><b>java.version, java.vendor, java.home, java.vm.version, java.vm.vendor, java.class.path</b> 
	 * 	Java virtual machine version, classpath and type.
	 * <li><b>os.name, os.arch, os.version</b> The operating system type/version.
	 * <li><b>user.name, user.home, user.dir</b> Data about the user the process is running as.
	 * <li><b>thread.list</b> A list of threads the Liric process is running.
	 * </ul>
	 * @see #serverConnectionThread
	 * @see #hashTable
	 * @see ExecuteCommand#run
	 * @see LiricStatus#getLogLevel
	 */
	private void getFullStatus()
	{
		ExecuteCommand executeCommand = null;
		Runtime runtime = null;
		StringBuffer sb = null;
		Thread threadList[] = null;
		int threadCount;

		// log level
		hashTable.put("Log Level",new Integer(status.getLogLevel()));
		// execute 'df -k' on instrument computer
		executeCommand = new ExecuteCommand("df -k");
		executeCommand.run();
		if(executeCommand.getException() == null)
			hashTable.put("Disk Usage",new String(executeCommand.getOutputString()));
		else
			hashTable.put("Disk Usage",new String(executeCommand.getException().toString()));
		// execute "ps -e -o pid,pcpu,vsz,ruser,stime,time,args" on instrument computer
		executeCommand = new ExecuteCommand("ps -e -o pid,pcpu,vsz,ruser,stime,time,args");
		executeCommand.run();
		if(executeCommand.getException() == null)
			hashTable.put("Process List",new String(executeCommand.getOutputString()));
		else
			hashTable.put("Process List",new String(executeCommand.getException().toString()));
		// execute "uptime" on instrument computer
		executeCommand = new ExecuteCommand("uptime");
		executeCommand.run();
		if(executeCommand.getException() == null)
			hashTable.put("Uptime",new String(executeCommand.getOutputString()));
		else
			hashTable.put("Uptime",new String(executeCommand.getException().toString()));
		// get vm memory situation
		runtime = Runtime.getRuntime();
		hashTable.put("Free Memory",new Long(runtime.freeMemory()));
		hashTable.put("Total Memory",new Long(runtime.totalMemory()));
		// get some java vm information
		hashTable.put("java.version",new String(System.getProperty("java.version")));
		hashTable.put("java.vendor",new String(System.getProperty("java.vendor")));
		hashTable.put("java.home",new String(System.getProperty("java.home")));
		hashTable.put("java.vm.version",new String(System.getProperty("java.vm.version")));
		hashTable.put("java.vm.vendor",new String(System.getProperty("java.vm.vendor")));
		hashTable.put("java.class.path",new String(System.getProperty("java.class.path")));
		hashTable.put("os.name",new String(System.getProperty("os.name")));
		hashTable.put("os.arch",new String(System.getProperty("os.arch")));
		hashTable.put("os.version",new String(System.getProperty("os.version")));
		hashTable.put("user.name",new String(System.getProperty("user.name")));
		hashTable.put("user.home",new String(System.getProperty("user.home")));
		hashTable.put("user.dir",new String(System.getProperty("user.dir")));
	}
}
