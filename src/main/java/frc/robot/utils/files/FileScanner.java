package frc.robot.utils.files;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * A utility class with various functionalites that can be used for file manipulation
 */
public class FileScanner
{
    /**
     * Searches for a folder/file from the robot deploy directory
     * 
     * @param subdirectory
     *            The file path relative to the deploy directory
     * @return The requested folder/file
     */
    public static File getFileFromDeploy(String subdirectory)
    {
        return new File(Filesystem.getDeployDirectory(), subdirectory);
    }

    /**
     * Gets all the file names from a certain folder
     * 
     * @param folder
     *            The folder containing the desired files
     * @return A list of all the file names as strings (extension included)
     */
    public static List<String> getFileNames(File folder)
    {
        return Arrays.asList(folder.list());
    }

    /**
     * Gets all the file names from a certain folder if it is a certain file type
     * 
     * @param folder
     *            The folder containing the desired files
     * @param fileExtension
     *            The file type of the files (insert as ".filetype")
     * @return A list of all the file names as strings (extension excluded)
     */
    public static List<String> getFileNamesFromType(File folder, String fileExtension)
    {

        String[] files = folder.list(new ExtensionFilter(fileExtension));
        List<String> fileNames = new ArrayList<String>();

        for (String file : files)
        {
            fileNames.add(file.replace(fileExtension, ""));
        }

        return fileNames;
    }

    /**
     * Gets all the files from a specified folder if it is a specified file type
     * 
     * @param folder
     *            The folder containing the desired files
     * @param fileExtension
     *            The file type of the files (insert as ".filetype")
     * @return A list of all every {@link File}
     */
    public static List<File> getFileFromType(File folder, String fileExtension)
    {

        return Arrays.asList(folder.listFiles(new ExtensionFilter(fileExtension)));
    }

    /**
     * Gets all the files from a specified folder
     * 
     * @param folder
     *            The folder containing the desired files
     * @return A list of all every {@link File}
     */
    public static List<File> getFileFromFolder(File folder)
    {

        return Arrays.asList(folder.listFiles());
    }

    /**
     * Applies any desired function that takes in a file name. Applies the function to
     * every single file in a folder of a certain type.
     * 
     * @param folder
     *            The folder containing the desired files
     * @param fileExtension
     *            The file type of the files (insert as ".filetype")
     * @param function
     *            The function that requires the file name (without the extension)
     * @param includeExtension
     *            Whether or not the extension should remain in the function input
     */
    public static void applyStringFunctionByType(File folder, String fileExtension,
        Consumer<String> function, boolean includeExtension)
    {
        String[] files = folder.list(new ExtensionFilter(fileExtension));

        for (String file : files)
        {
            function.accept(includeExtension ? file : file.replace(fileExtension, ""));
        }
    }

    /**
     * Applies any desired function that takes in a file name. Applies the function to
     * every single file in the folder.
     * 
     * @param folder
     *            The folder containing the desired files
     * @param function
     *            The function that requires the File
     * @param includeExtension
     *            Whether or not the extension should remain in the function input
     */
    public static void applyStringFunctionByFolder(File folder, Consumer<String> function,
        boolean includeExtension)
    {
        applyStringFunctionByType(folder, "", function, includeExtension);
    }

    /**
     * Applies any desired function that takes in a {@link File}. Applies the function to
     * every single file in a folder of a certain type.
     * 
     * @param folder
     *            The folder containing the desired files
     * @param fileExtension
     *            The file type of the files (insert as ".filetype")
     * @param function
     *            The function that requires the File
     */
    public static void applyFileFunctionByType(File folder, String fileExtension,
        Consumer<File> function)
    {
        File[] files = folder.listFiles(new ExtensionFilter(fileExtension));

        for (File file : files)
        {
            function.accept(file);
        }
    }

    /**
     * Applies any desired function that takes in a File. Applies the function to every
     * single file in the folder.
     * 
     * @param folder
     *            The folder containing the desired files
     * @param function
     *            The function that requires the File
     */
    public static void applyFileFunctionByFolder(File folder, Consumer<File> function)
    {
        applyFileFunctionByType(folder, "", function);
    }
}
