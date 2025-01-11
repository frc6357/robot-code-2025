package frc.robot.utils.files;

import java.io.File;
import java.io.FilenameFilter;

/**
 * A filter type used by the {@link File} class to pick out all files of a certain
 * filetype/extension
 */
public class ExtensionFilter implements FilenameFilter
{
    private String[] extension;

    /**
     * Creates a new filter that looks for a specific extension
     * 
     * @param extension
     *            The desired file types (put as ".filetype" and put multiple file types
     *            separately)
     */
    public ExtensionFilter(String... extension)
    {
        this.extension = extension;
    }

    @Override
    public boolean accept(File file, String name)
    {
        for (String ext : extension)
        {
            if (name.endsWith(ext))
            {
                return true;
            }
        }
        return false;
    }

}