

/**
 *
 * @author bruce
 */

import static java.lang.System.*;
import java.util.Scanner;


public class TestString2 {
    public static void main(String[] args)
    {
//        Scanner sc= new Scanner(in);
//        //sc.useDelimiter("\n");
//        
//        if (sc.hasNext())
//        {
//            String str=sc.next();
//        
//            
//            
//            
//            
//        }
        
        char[] src={'P','a','s','i','r'};
        out.print("src="); printCharArray(src);
        
        char[] reverse=reverse(src);
        out.print("reverse="); printCharArray(reverse);
        
        out.print("permutation=\n");
        permute(src);
        out.println("totally "+ countPermutation+ " situations");
        
    }
    
    private static int countPermutation=0;
    
    public static void permute(char[] src)
    {
        int len=src.length;
        boolean[] used=new boolean[len];
        int level=0;
        char[] permutation=new char[len];
        
        permuting(src, used, level, len, permutation);

    }
    
    public static void permuting(final char[] src, 
                                 boolean[] used, 
                                 int level, 
                                 final int len,
                                 char[] permutation)
    {
        if (level>=len)
        {
            printCharArray(permutation);
            countPermutation++;
            return;
        }
        
        for (int i=0; i<len; ++i)
        {
            if (used[i]==true)
                continue;
            else
            {
                permutation[level]=src[i];
                
                used[i]=true; 
                level++;
                
                permuting(src, used, level, len, permutation);
                
                level--;
                used[i]=false;
            }
            
            
        }
        
        
    }
    
    
    public static char[] reverse(char[] src)
    {
        int len=src.length;
        char[] reverse=new char[len];
        for (int i=0; i<len; ++i)
            reverse[i]=src[len-i-1];
        return reverse;
    }
    
    public static void printCharArray(char[] src)
    {
        for (int i=0; i<src.length; ++i)
            out.print(src[i]);    
        out.println();      
    }
    
    
    
}
