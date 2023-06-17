/*
This example is used in a tutorial on FTC myBlocks.
It shows how one myBlock can write a number to a file on the RC phone
or Control Hub, and another myBlock can read that number from the file.

This is not possible with regular Blocks, in FTC app version 6.1.

This example assumes a team wants to store and retrieve a *number*, not text.
The file operations shown here are intended for storing and retrieving
text information. So this example stores a number as text, requiring conversion
when writing and when reading.

If a team instead wants to store an actual text string, this example could be
simplified (conversions not needed).

A challenge for the student: write and read *multiple* values, that might
be used for robot set-up, calibration, or choices of autonomous program.


Note 1: The method getSettingsFile() retrieves the settings (including location)
        of the named file.  If the file doesn't already exist, it is created
        in the FIRST/settings folder.  Put the filename in quotes if it's not
        already a declared variable of type String.

Note 2: There is also a method copyFile(File fromFile, File toFile).

Note 3: The method String.valueOf() reads a numerical value as a String.

Note 4: The parseDouble() method interprets a String value as a double.
        The trim() method removes any leading or trailing white space. 

Note 5: The write and read myBlocks can omit the filename parameter, if a team
        always uses the same file.  In such case the filename can be declared
        once at the class level (must be static), and used by all myBlock
        methods.  Like this:
        static File myFileName = AppUtil.getInstance().getSettingsFile("myTestFile.txt");

Note 6: The class ReadWriteFile does not appear to have a method for 
        appending to a file.  Might need to use java.io.Writer.write() or a 
        java.io.FileWriter method.
*/

package org.firstinspires.ftc.teamcode.util.CycleTracker;

// these are (usually!) added automatically by OnBotJava when needed

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
//NO WORK
public class SampleMyBlock {
static Telemetry telemetry;
    public SampleMyBlock(Telemetry telemetry){
        this. telemetry = telemetry;
    }
    // This Annotation must appear immediately before any myBlock method.
    // It's optional to add a comment, tooltip, and/or parameterLabels.
    // Comment must appear on a single line, no rollovers.
//    @ExportToBlocks (
//    comment = "Writes a number to specified file on RC device. Includes telemetry.",
//    tooltip = "Write number to file on RC device.",
//    parameterLabels = {"Number to Write", "Full Filename (.txt)"}
//    )
    // This myBlock method writes a number (as text) to a file.
    // It has 2 inputs and no outputs (keyword void).
    public static void writeToFile (double myNumber, String toFileName) {
        
        // Using the properties of the specified "to" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(toFileName);

        // Write the provided number to the newly declared filename.
        // See Note 3 above.
        ReadWriteFile.writeFile(myFileName, String.valueOf(myNumber));
        
        telemetry.addData("Filename", toFileName);
        telemetry.addData("Number being written", myNumber);
        telemetry.update();         // display info on Driver Station screen
        
    }   // end of method writeToFile()



}   // end of class SampleMyBlock
