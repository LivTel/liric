// StatusExposureLengthCommand.java
// $Id$
package ngat.liric.command;

import java.io.*;
import java.lang.*;
import java.net.*;

/**
 * The "status exposure length" command is an extension of the IntegerReplyCommand, and returns the 
 * per-frame exposure length in the current multrun.
 * This status is available per C layer.
 * @author Chris Mottram
 * @version $Revision$
 */
public class StatusExposureLengthCommand extends IntegerReplyCommand implements Runnable
{
	/**
	 * Revision Control System id string, showing the version of the Class.
	 */
	public final static String RCSID = new String("$Id$");
	/**
	 * The command to send to the server.
	 */
	public final static String COMMAND_STRING = new String("status exposure length");

	/**
	 * Default constructor.
	 * @see IntegerReplyCommand
	 * @see #commandString
	 * @see #COMMAND_STRING
	 */
	public StatusExposureLengthCommand()
	{
		super();
		commandString = COMMAND_STRING;
	}

	/**
	 * Constructor.
	 * @param address A string representing the address of the server, i.e. "liric",
	 *     "localhost"
	 * @param portNumber An integer representing the port number the server is receiving command on.
	 * @see IntegerReplyCommand
	 * @see #COMMAND_STRING
	 * @exception UnknownHostException Thrown if the address in unknown.
	 */
	public StatusExposureLengthCommand(String address,int portNumber) throws UnknownHostException
	{
		super(address,portNumber,COMMAND_STRING);
	}

	/**
	 * Get the exposure length in the current multrun. 
	 * This is either the per-frame length of one exposure in milliseconds.
	 * @return An integer, the exposure length in milliseconds. 
	 * @exception Exception Thrown if getting the data fails, either the run method failed to communicate
	 *         with the server in some way, or the method was called before the command had completed.
	 */
	public int getExposureLength() throws Exception
	{
		if(parsedReplyOk)
			return super.getParsedReplyInteger();
		else
		{
			if(runException != null)
				throw runException;
			else
				throw new Exception(this.getClass().getName()+":getExposureLength:Unknown Error.");
		}
	}

	/**
	 * Main test program.
	 * @param args The argument list.
	 */
	public static void main(String args[])
	{
		StatusExposureLengthCommand command = null;
		String hostname = null;
		int portNumber = 8284;

		if(args.length != 2)
		{
			System.out.println("java ngat.liric.command.StatusExposureLengthCommand <hostname> <port number>");
			System.exit(1);
		}
		try
		{
			hostname = args[0];
			portNumber = Integer.parseInt(args[1]);
			command = new StatusExposureLengthCommand(hostname,portNumber);
			command.run();
			if(command.getRunException() != null)
			{
				System.err.println("StatusExposureLengthCommand: Command failed.");
				command.getRunException().printStackTrace(System.err);
				System.exit(1);
			}
			System.out.println("Finished:"+command.getCommandFinished());
			System.out.println("Reply Parsed OK:"+command.getParsedReplyOK());
			System.out.println("Exposure Length(ms):"+command.getExposureLength());
		}
		catch(Exception e)
		{
			e.printStackTrace(System.err);
			System.exit(1);
		}
		System.exit(0);
	}
}
