
/**
 * Find the first non-repeated character
 * @author bruce
 */

import static java.lang.System.*;
import java.util.Map;
import java.util.Map.Entry; //interface(Entry<K,V>) of interface(Map<K,V>) 
import java.util.HashMap;
//import java.util.LinkedHashMap;
import java.util.Hashtable;
import java.util.Scanner;
import java.util.Iterator;

public class TestHash {
    
    public static Character firstNonRe(String str)
    {
        HashMap<Character, Integer> counter=new HashMap<Character, Integer>();
        //Hashtable<Character, Integer> counter=new Hashtable<Character, Integer>();
        
        for (int i=0; i<str.length(); ++i)
        {
            if (counter.get(str.charAt(i))== null)
            {    
                //out.println(str.charAt(i)+"--->"+counter.get(str.charAt(i)));
                counter.put(str.charAt(i), 1);
            }
            else
                counter.put(str.charAt(i), counter.get(str.charAt(i))+1  );
        }
        
        
//        Iterator< Character > itr = counter.keySet().iterator();
//        Character temp;
//        while (itr.hasNext())
//        {       
//            out.println((temp=itr.next()) + ":" + counter.get(temp) );
//        }
        
        
        Iterator< Entry<Character,Integer> > itr2 = counter.entrySet().iterator();
        Character temp2;
        while (itr2.hasNext())
        {       
            out.println((temp2=itr2.next().getKey()) + ":" + counter.get(temp2) );           
        }
        
        
        
        
        for (int i=0; i<str.length(); ++i)
            if ( counter.get(str.charAt(i)) ==1 ) return str.charAt(i);
        
        return null;
    }
    
    
    public static void main(String[] args)
    {
        Scanner scanner= new Scanner(in);
        scanner.useDelimiter("\n");
        
        String src;
        if (scanner.hasNext())        
        {
            src=scanner.next();    
            //out.print("src="+src+"-->");

            out.println("The first non-repeated character is:"+firstNonRe(src));
        }
    }
    
    
}
