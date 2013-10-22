/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author bruce
 */
import static java.lang.System.*;
import java.io.IOException;


public class TestString {
    public String reverse(String src)
    {
        out.println("src.length()="+ src.length());
        StringBuffer dst= new StringBuffer(src.length());
        for (int i=src.length()-1; i>=0; --i)
            dst.append(src.charAt(i));
            
        return new String(dst);
    }
    
    public static void main(String args[]) throws IOException
    {
        byte buffer[] = new byte[128];   
        int count=System.in.read(buffer); // also reading '\n'
        
        System.out.println("count= "+count);
        for (int i=0; i<count; ++i)
            System.out.print(" "+ buffer[i]);
        
        System.out.println();
        for (int i=0; i<count; ++i)
            System.out.print(" "+ (char)buffer[i]);
        
        System.out.println("\n~~~~~~~~~~~~~~~~~~~~~~~~");
        
        /*
        String src= new String(buffer);
        out.println("buffer.length="+buffer.length); // output:128
        out.println("src.length()="+src.length());   // output:128
        */
        
        String src = new String(buffer);
        /*
        if (buffer[count-1]=='\n')
        {
            out.println("delete the ending \\n");           
            String[] result=src.split("\n");
            src=result[0];
        }
        else
        {
            StringBuffer strB= new StringBuffer(count); 
            for (int i=0; i<count; ++i)
                strB.append(src.charAt(i));
            
            src=new String(strB);
        }
        */
        int len=count;
        if(buffer[count-1]=='\n')
        {
            out.println("delete the ending \\n");
            len--;
        }
        StringBuffer strB= new StringBuffer(len); 
        for (int i=0; i<len; ++i)
            strB.append(src.charAt(i));
            
        src=new String(strB);
            
        
        System.out.println(src);
        
        String dst= new TestString().reverse(src);      
        System.out.println(dst);
        
    }
}
