package org.usfirst.frc.team4028.robot;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class LogData 
{
	private List<String> _names;
	private List<String> _values;
	
	
	public LogData()
	{
		_names = new ArrayList<String>();
		_values = new ArrayList<String>();
	}
		
	public void AddData(String name, String value)
	{
		_names.add(name);
		_values.add(value);
	}
	
	public void ResetData()
	{
		_names = new ArrayList<String>();
		_values = new ArrayList<String>();	
	}

	// build a TSV string for the header row
	public String BuildTSVHeader()
	{
		return BuildTSVString(_names);
	}

	// build a TSV string for a data row
	public String BuildTSVData()
	{
		return BuildTSVString(_values);
	}
	
	// build a TSV string from a List<string>
	private String BuildTSVString(List<String> myList)
	{
		StringBuilder sb = new StringBuilder();
		
		for(String item : myList)
		{
			sb.append(item + "\t");
		}
		
		String lineToWrite = sb.toString();
		
		// remove the trailing tab
		lineToWrite = lineToWrite.substring(0, lineToWrite.length() - 1);
		
		// add trailing crlf
		lineToWrite = lineToWrite + "\r\n";
		
		return lineToWrite;
	}
}
