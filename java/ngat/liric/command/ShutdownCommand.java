// ShutdownCommand.java
// $Id$
package ngat.liric.command;

import java.io.*;
import java.lang.*;
import java.net.*;
import java.util.*;

/**
 * The "shutdown" command is an extension of the Command, and exits the C layer process.
 * @author Chris Mottram
 * @version $Revision$
 */
public class ShutdownCommand extends Command implements Runnable
{
	/**
	 * Revision Control System id string, showing the version of the Class.
	 */
	public final static String RCSID = new String("$Id$");
	/**
	 * The command to send to the server.
	 */
	public final static String COMMAND_STRING = new String("shutdown");

	/**
	 * Default constructor.
	 * @see Command
	 * @see #commandString
	 * @see #COMMAND_STRING
	 */
	public ShutdownCommand()
	{
		super();
		commandString = COMMAND_STRING;
	}

	/**
	 * Constructor.
	 * @param address A string representing the address of the server, i.e. "liric",
	 *     "localhost"
	 * @param portNumber An integer representing the port number the server is receiving command on.
	 * @see Command
	 * @see Command#setAddress
	 * @see Command#setPortNumber
	 * @exception UnknownHostException Thrown if the address in unknown.
	 */
	public ShutdownCommand(String address,int portNumber) throws UnknownHostException
	{
		super(address,portNumber,COMMAND_STRING);
	}

	/**
	 * Main test program.
	 * @param args The argument list.
	 */
	public static void main(String args[])
	{
		ShutdownCommand command = null;
		String hostname = null;
		int portNumber = 8284;

		if((args.length != 2))
		{
			System.out.println("java ngat.liric.command.ShutdownCommand <hostname> <port number>");
			System.exit(1);
		}
		try
		{
			hostname = args[0];
			portNumber = Integer.parseInt(args[1]);
			command = new ShutdownCommand(hostname,portNumber);
			command.run();
			if(command.getRunException() != null)
			{
				System.err.println("ShutdownCommand: Command failed.");
				command.getRunException().printStackTrace(System.err);
				System.exit(1);
			}
			System.out.println("Finished:"+command.getCommandFinished());
			System.out.println("Reply Parsed OK:"+command.getParsedReplyOK());
			System.out.println("Return Code:"+command.getReturnCode());
			System.out.println("Reply String:"+command.getParsedReply());
		}
		catch(Exception e)
		{
			e.printStackTrace(System.err);
			System.exit(1);
		}
		System.exit(0);
	}
}
