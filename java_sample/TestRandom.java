/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author bruce
 */
import java.util.Collection;
import java.util.Collections;
import java.util.Set;
import java.util.List;
import java.util.LinkedList;

import java.util.concurrent.RecursiveTask;


import java.lang.Math;

public class TestRandom {
    public static void main(String[] args)
    {
        //[20,50) decimal
//        for (int i=0; i<10; ++i)
//            System.out.println(Math.random()*30+20);
        
        //[20, 30] integer
        for (int i=0; i<15; ++i)
            System.out.println(  (int)(Math.random()*11+20) );
        
        
        // Tell the difference of the following 2 ways
        List<String> ll=new LinkedList<String>();
        LinkedList<String> ll2=new LinkedList<String>();
        
        ll2.add(null);     //OK
        ll.add(null);      //OK
        ll2.getFirst();    //OK
        //ll.getFirst();   //Wrong, because getFirst() is exclusively owned by LinkedList
    }
    
    
    
}
