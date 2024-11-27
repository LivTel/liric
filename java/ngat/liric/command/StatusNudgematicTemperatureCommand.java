// StatusNudgematicTemperatureCommand.java
// $Id$
package ngat.liric.command;

import java.io.*;
import java.lang.*;
import java.net.*;
import java.util.*;

/**
 * The "status nudgematic temperature" command is an extension of Command, and returns the 
 * current envionmental temperatures measured by the Nudgematic Arduino (in 3 positions).
 * @author Chris Mottram
 * @version $Revision$
 */
public class StatusNudgematicTemperatureCommand extends Command implements Runnable
{
	/**
	 * Revision Control System id string, showing the version of the Class.
	 */
	public final static String RCSID = new String("$Id$");
	/**
	 * The command to send to the server.
	 */
	public final static String COMMAND_STRING = new String("status nudgematic temperature");
	/**
	 * Number, passed as an argument to getNudgematicTemperature, to specify returning temperature 1 
	 * (external case temperature).
	 */
	public final static int TEMP_EXTERNAL_CASE = 1;
	/**
	 * Number, passed as an argument to getNudgematicTemperature, to specify returning temperature 2 
	 * (radiator block temperature).
	 */
	public final static int TEMP_RADIATOR_BLOCK = 2;
	/**
	 * Number, passed as an argument to getNudgematicTemperature, to specify returning temperature 3 
	 * (internal case temperature).
	 */
	public final static int TEMP_INTERNAL_CASE = 3;
	/**
	 * The parsed reply timestamp.
	 */
	protected Date parsedReplyTimestamp = null;
	/**
	 * The parsed reply temperature 1 (external case temperature) (in degrees C).
	 */
	protected double parsedReplyTemperature1 = 0.0;
	/**
	 * The parsed reply temperature 2 (radiator block temperature) (in degrees C).
	 */
	protected double parsedReplyTemperature2 = 0.0;
	/**
	 * The parsed reply temperature 3 (internal block temperature) (in degrees C).
	 */
	protected double parsedReplyTemperature3 = 0.0;
	
	/**
	 * Default constructor.
	 * @see IntegerReplyCommand
	 * @see #commandString
	 * @see #COMMAND_STRING
	 */
	public StatusNudgematicTemperatureCommand()
	{
		super();
		commandString = COMMAND_STRING;
	}

	/**
	 * Constructor.
	 * @param address A string representing the address of the server, i.e. "liric",
	 *     "localhost", "192.168.1.34"
	 * @param portNumber An integer representing the port number the server is receiving command on.
	 * @see IntegerReplyCommand
	 * @see #COMMAND_STRING
	 * @exception UnknownHostException Thrown if the address in unknown.
	 */
	public StatusNudgematicTemperatureCommand(String address,int portNumber) throws UnknownHostException
	{
		super(address,portNumber,COMMAND_STRING);
	}

	/**
	 * Parse a string returned from the server over the telnet connection.
	 * In this case it is of the form: 
	 * '&lt;n&gt; &lt;YYYY-mm-ddTHH:MM:SS.sss&gt; &lt;TZ&gt; &lt;n.nnn&gt; &lt;n.nnn&gt; &lt;n.nnn&gt;'
	 * The first number is a success failure code, if it is zero a timestamp and 3 temperatures follow.
	 * @exception Exception Thrown if a parse error occurs.
	 * @see #replyString
	 * @see #parsedReplyString
	 * @see #parsedReplyOk
	 * @see #parsedReplyTimestamp
	 * @see #parsedReplyTemperature1
	 * @see #parsedReplyTemperature2
	 * @see #parsedReplyTemperature3
	 */
	public void parseReplyString() throws Exception
	{
		Calendar calendar = null;
		TimeZone timeZone = null;
		StringTokenizer st = null;
		String timeStampDateTimeString = null;
		String timeStampTimeZoneString = null;
		String temperatureString1 = null;
		String temperatureString2 = null;
		String temperatureString3 = null;
		double second=0.0;
		int sindex,tokenIndex,day=0,month=0,year=0,hour=0,minute=0;
	
		super.parseReplyString();
		if(parsedReplyOk == false)
		{
			parsedReplyTimestamp = null;
			parsedReplyTemperature1 = 0.0;
			parsedReplyTemperature2 = 0.0;
			parsedReplyTemperature3 = 0.0;
			return;
		}
		st = new StringTokenizer(parsedReplyString," ");
		tokenIndex = 0;
		while(st.hasMoreTokens())
		{
			if(tokenIndex == 0)
				timeStampDateTimeString = st.nextToken();
			else if(tokenIndex == 1)
				timeStampTimeZoneString = st.nextToken();
			else if(tokenIndex == 2)
				temperatureString1 = st.nextToken();
			else if(tokenIndex == 3)
				temperatureString2 = st.nextToken();
			else if(tokenIndex == 4)
				temperatureString3 = st.nextToken();
			tokenIndex++;
		}// end while
		// timeStampDateTimeString should be of the form: YYYY-mm-ddTHH:MM:SS.sss
		st = new StringTokenizer(timeStampDateTimeString,"-T:");
		tokenIndex = 0;
		while(st.hasMoreTokens())
		{
			if(tokenIndex == 0)
				year = Integer.parseInt(st.nextToken());// year including century
			else if(tokenIndex == 1)
				month = Integer.parseInt(st.nextToken());// 01..12
			else if(tokenIndex == 2)
				day = Integer.parseInt(st.nextToken());// 0..31
			else if(tokenIndex == 3)
				hour = Integer.parseInt(st.nextToken());// 0..23
			else if(tokenIndex == 4)
				minute = Integer.parseInt(st.nextToken());// 00..59
			else if(tokenIndex == 5)
				second = Double.parseDouble(st.nextToken());// 00..61 + milliseconds as decimal
			tokenIndex++;
		}// end while
		// parse the timezone string timeStampTimezoneString
		timeZone = TimeZone.getTimeZone(timeStampTimeZoneString);
		// create calendar
		calendar = Calendar.getInstance();
		calendar.setTimeZone(timeZone);
		// set calendar
		calendar.set(year,month-1,day,hour,minute,(int)second);// month is zero-based.
		// get timestamp from calendar 
		parsedReplyTimestamp = calendar.getTime();
		// parse temperature1
		parsedReplyTemperature1 = Double.parseDouble(temperatureString1);
		// parse temperature2
		parsedReplyTemperature2 = Double.parseDouble(temperatureString2);
		// parse temperature3
		parsedReplyTemperature3 = Double.parseDouble(temperatureString3);
	}

	/**
	 * Get the specified environmental temperature of Liric, from the Nudgematic, at the specified timestamp.
	 * @param sensorNumber Which Nudgematic environmental sesnor to return the value for, there are three:
	 *      	TEMP_EXTERNAL_CASE,  TEMP_RADIATOR_BLOCK, TEMP_INTERNAL_CASE
	 * @return A double, the temperature (in degrees centigrade).
	 * @exception Exception Thrown if getting the data fails, either the run method failed to communicate
	 *         with the server in some way, or the method was called before the command had completed.
	 * @see #TEMP_EXTERNAL_CASE
	 * @see #TEMP_RADIATOR_BLOCK
	 * @see #TEMP_INTERNAL_CASE
	 * @see #parsedReplyOk
	 * @see #runException
	 * @see #parsedReplyTemperature1
	 * @see #parsedReplyTemperature2
	 * @see #parsedReplyTemperature3
	 */
	public double getNudgematicTemperature(int sensorNumber) throws Exception
	{
		if(parsedReplyOk)
		{
			if(sensorNumber == TEMP_EXTERNAL_CASE)
			   return parsedReplyTemperature1;
			else if(sensorNumber == TEMP_RADIATOR_BLOCK)
			   return parsedReplyTemperature2;
			else if(sensorNumber == TEMP_INTERNAL_CASE)
			   return parsedReplyTemperature3;
			else
			{
				throw new IllegalArgumentException(this.getClass().getName()+
								   ":getNudgematicTemperature:Sensor Number '"+
								   sensorNumber+"'was out of range.");
			}
		}
		else
		{
			if(runException != null)
				throw runException;
			else
				throw new Exception(this.getClass().getName()+":getNudgematicTemperature:Unknown Error.");
		}
	}

	/**
	 * Get the timestamp representing the time the nudgematic enviromental temperatures were measured.
	 * @return A date, the time the nudgematic enviromental temperatures were measured.
	 * @exception Exception Thrown if getting the data fails, either the run method failed to communicate
	 *         with the server in some way, or the method was called before the command had completed.
	 * @see #parsedReplyOk
	 * @see #runException
	 * @see #parsedReplyTimestamp
	 */
	public Date getTimestamp() throws Exception
	{
		if(parsedReplyOk)
		{
			return parsedReplyTimestamp;
		}
		else
		{
			if(runException != null)
				throw runException;
			else
				throw new Exception(this.getClass().getName()+":getTimestamp:Unknown Error.");
		}
	}

	/**
	 * Main test program.
	 * @param args The argument list.
	 */
	public static void main(String args[])
	{
		StatusNudgematicTemperatureCommand command = null;
		String hostname = null;
		int portNumber = 8284;

		if(args.length != 2)
		{
			System.out.println("java ngat.liric.command.StatusNudgematicTemperatureCommand <hostname> <port number>");
			System.exit(1);
		}
		try
		{
			hostname = args[0];
			portNumber = Integer.parseInt(args[1]);
			command = new StatusNudgematicTemperatureCommand(hostname,portNumber);
			command.run();
			if(command.getRunException() != null)
			{
				System.err.println("StatusNudgematicTemperatureCommand: Command failed.");
				command.getRunException().printStackTrace(System.err);
				System.exit(1);
			}
			System.out.println("Finished:"+command.getCommandFinished());
			System.out.println("Reply Parsed OK:"+command.getParsedReplyOK());
			System.out.println("Nudgematic Temperature 1 (external case temperature):"+
					   command.getNudgematicTemperature(TEMP_EXTERNAL_CASE));
			System.out.println("Nudgematic Temperature 2 (radiator block temperature):"+
					   command.getNudgematicTemperature(TEMP_RADIATOR_BLOCK));
			System.out.println("Nudgematic Temperature 3 (internal case temperature):"+
					   command.getNudgematicTemperature(TEMP_INTERNAL_CASE));
			System.out.println("At Timestamp:"+command.getTimestamp());
		}
		catch(Exception e)
		{
			e.printStackTrace(System.err);
			System.exit(1);
		}
		System.exit(0);
	}
}
