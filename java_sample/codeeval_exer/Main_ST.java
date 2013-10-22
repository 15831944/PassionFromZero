/* *****************************************
 * ***** Solution of "Discount Offers" *****
 * *****************************************
 * Problem Description: https://www.codeeval.com/public_sc/48/
 * @author: Haiyue (Bruce) Li
 * @date: 09/20/2013
 * @email: jkzuoluo19@gmail.com, haiyuel@andrew.cmu.edu
 * 
 * [How To Run]
 * ********************************
 * ***** ->javac Main_ST.java    *****
 * ***** ->java Main_ST data     *****
 * ********************************
 * ("data": the file name of the ASCII-encoded file )
 * 
 * 
 * Copyright (c) 2013, Haiyue Li. All rights reserved.
 */

//package codeeval_exer;

import static java.lang.System.*;
import java.lang.Math;
import java.io.File;
import java.util.Scanner;
//import java.util.Vector;
import java.util.ArrayList;


import java.io.FileNotFoundException;


public class Main_ST //Product to Customer
{ 
    public static void main(String[] args) throws FileNotFoundException
    {        
        long starttime= System.currentTimeMillis();
        
        // Calcute Maximum SS for each case
        Main_ST p2c= new Main_ST();
        p2c.calcMaximumSS(new File(args[0]));
        
        long endtime= System.currentTimeMillis();
        out.println("Running took "+(endtime-starttime)+" ms");
    }
    
    public void calcMaximumSS(File file) throws FileNotFoundException
    {
        Scanner scanner = new Scanner(file);
        
        //--- Read all the cases line by line
        /*
         * Vector increases capacity by twice, if not setup capacityIncrement
         * Vector's add() is 'synchronized', system costs
         */  
        //Vector<String> cases = new Vector<String>(50, 10);
        
        /*
         * ArrayList increases capacity by 1.5 times
         * ArrayList's add() is NOT 'synchronized'
         */
        ArrayList<String> cases = new ArrayList<String>(50);
        
        String line;
        while (scanner.hasNextLine())
        {
            line=scanner.nextLine();
            // record the non-empty line
            if (line.compareTo("")!=0)
                cases.add(line);
        }
        /*        
        //DEBUG
        out.println(cases.size());
        for (int i=0; i<cases.size(); ++i)
            out.println(cases.get(i));
        */
        
        //--- For each and every case, calculate the max combined suitability score
        double[] result= new double[cases.size()];
        
        String[][] caseInfo=new String[2][]; // one row for customers
                                             // the other row for products   
        for (int i=0; i<cases.size(); ++i)
        {
            //=== Check whether this case is good to go
            if ( !caseCheck(cases.get(i), caseInfo) )
            {
                out.printf("%.2f\n", result[i]);
                continue;
            }                
            String[] customers=caseInfo[0];
            String[] products=caseInfo[1];
            /*
            //DEBUG
            out.println(customers.length);
            for (String customer: customers)   
                out.println(customer);
            
            out.println(products.length);
            for (String product: products)
                out.println(product);
            */
            
            //=== Calculate Suitability Score
            double[][] SS= calcSuitabilityScore(customers, products);
            /*
            //DEBUG
            out.println("Case "+ (i+1)+":");
            for (int row=0; row<SS.length; ++row)
            {
                for (int col=0; col<SS[row].length; ++col)
                {
                    out.print(SS[row][col]+",\t");
                }
                out.println();
            }
            out.println();
            */
            
            //=== Enumerate all the permutation of allocation, record the max combined SS  
            result[i]= permuteSS(SS);
            out.printf("%.2f\n", result[i]);
        }
           
    }
    
    private static boolean caseCheck(String oneCase, String[][] caseInfo)
    {
        //=== Split the case by ';'
        String[] caseSplit = oneCase.split(";");
        //CHECK
        //out.println("caseSplit.length="+caseSplit.length);
        if (caseSplit.length!=2 ||
            caseSplit[0].compareTo("")==0 || caseSplit[1].compareTo("")==0)
        {//Case format is not correct
            return false;
        }
            
        //=== analyze the case by checking ','
        caseInfo[0]=analyze(caseSplit[0]);
        caseInfo[1]=analyze(caseSplit[1]);
        //CHECK
        if (caseInfo[0][0].compareTo("")==0 ||
            caseInfo[1][0].compareTo("")==0)
        {//No valid elements
            return false;
        }
        
        
        return true;
    }
    
    /**
     * Split the str by ',' intelligently
     * @param str
     * @return all the subStrings which are not ""
     */
    private static String[] analyze(String str)
    {
        ArrayList<String> output= new ArrayList<String>();
        
        int off=0;   //offset
        int next=0;        
        while ( (next=str.indexOf(',', off))!=-1 )
        {
            if (next==off)
            {//delete the ""
                off=next+1;
                continue;
            }
            
            output.add(str.substring(off, next));
            off=next+1;           
        }
        
        // No ',' at all
        if (off==0)
            return new String[]{str};
        
        // Handle the last subString
        if ( off!=str.length() )
            output.add(str.substring(off, str.length()));
        
        // No valid char at all (all of the chars are ',')   
        if (output.isEmpty())
            return new String[]{""};

       
        return output.toArray(new String[output.size()]);      
    }
      
    private static double[][] calcSuitabilityScore(final String[] customers, final String[] products)
    {
        int sizeOfCustomers=customers.length;
        int sizeOfProducts=products.length;
        
        double[][] SS= new double[sizeOfProducts][sizeOfCustomers];    
        /*
        //DEBUG
        for (String product: products)
            out.println(product+",\t\tletters:"+lettersOf(product));      
        
        for (String customer: customers)
        {
            out.print(customer+",\tletters:"+lettersOf(customer));
            out.print(",\tvowels:"+vowelsOf(customer));
            out.println(",\tconsonants:"+consonantsOf(customer));      
        }
        */
        
        //=== Count the number of letters towards each customer and each product
        int[] productsLetters= new int[sizeOfProducts];
        for (int i=0; i<sizeOfProducts; ++i)
            productsLetters[i]=lettersOf(products[i]);
            
        int[] customersLetters= new int[sizeOfCustomers];
        for (int i=0; i<sizeOfCustomers; ++i)
            customersLetters[i]=lettersOf(customers[i]);
        
        //=== Algorithm stated in the problem online
        for (int i=0; i<sizeOfProducts; ++i)
        {
            if ((productsLetters[i] % 2)==0)  //even
            {
                for (int j=0; j<sizeOfCustomers; ++j)
                {
                    SS[i][j] = vowelsOf(customers[j])*1.5;
                    if ( gcdOf(productsLetters[i], customersLetters[j])>1 )
                        SS[i][j]*=1.5;
                }
            }
            else //odd
            {
                for (int j=0; j<sizeOfCustomers; ++j)
                {
                    SS[i][j] = consonantsOf(customers[j]);
                    if ( gcdOf(productsLetters[i], customersLetters[j])>1 )
                        SS[i][j]*=1.5;
                }
            }       
        }
             
        return SS;
    }
    
    private static double permuteSS(double[][] SS)
    {   
        int numOfCustomers= SS[0].length;
        int numOfProducts= SS.length;
        
        //=== For passing reference, use an array
        double[] maxTotalSS={0.0};
        
        //=== Record whether or not the element in SS is used
        boolean[][] used= new boolean[numOfProducts][numOfCustomers];
        
        //=== Record which row is being chosen; record current sum of chosen SS
        int level=0;
        double sum=0.0;
        
        //=== Perform permuting
        if (numOfCustomers == numOfProducts) //Num of customers == Num of products
        {
            permutingR(SS, used, level, sum, maxTotalSS);
        }
        else if (numOfCustomers > numOfProducts) //Num of customers > Num of products
        {
            //=== permuting according to used[][], that is set bit by bit
            MaskInfo maskInfo=new MaskInfo(numOfCustomers, numOfCustomers-numOfProducts);
            permutingRowByRow(0, maskInfo, SS, used, maxTotalSS);
            //permutingRowByRow(maskInfo, SS, used, maxTotalSS); posRecord=-1;//reset posRecord
        }
        else //Num of customers < Num of products
        {
            MaskInfo maskInfo=new MaskInfo(numOfProducts, numOfProducts-numOfCustomers);
            permutingColByCol(0, maskInfo, SS, used, maxTotalSS);
        }       
        
        
        return maxTotalSS[0];
    }

    /**
     * Traverse all the situations of combined SS, record the max one
     * @param flatMatrix: 'flat' means matrix[0].length >= matrix.length
     * @param used: whether this SS has been chosen
     * @param level: which row of the SS[][] is being handled
     * @param sum: sum of SSs that have been chosen
     * @param maxTotalSS: output. The max total(combined) SS 
     */
    private static void permutingR(final double[][] flatMatrix, 
                                   boolean[][] used,
                                   int level,
                                   double sum,
                                   double[] maxTotalSS)
    {
        if (level==flatMatrix.length)
        {
            if (sum>maxTotalSS[0])
                maxTotalSS[0]=sum;
            
            return;
        }
        
        for (int i=0; i<flatMatrix[0].length; ++i)
        {
            if (!used[level][i])
            {
                //=== Mark the related column elements below
                for (int row=level; row<flatMatrix.length; ++row)
                    used[row][i]=true;
                
                //=== Set sum and level
                sum+=flatMatrix[level][i];
                level++;
                
                //=== Select next SS
                permutingR(flatMatrix, used, level, sum, maxTotalSS);
                
                //=== Reset level and sum
                level--;
                sum-=flatMatrix[level][i];
                
                //=== Unmark the related column elements below
                for (int row=level; row<flatMatrix.length; ++row)
                    used[row][i]=false;
            }           
        }
  
    }
    
    /**
     * Traverse all the situations of combined SS, record the max one
     * @param tallMatrix: 'tall' means matrix.length >= matrix[0].length
     * @param used: whether this SS has been chosen
     * @param level: which col of the SS[][] is being handled
     * @param sum: sum of SSs that have been chosen
     * @param maxTotalSS: output. The max total(combined) SS 
     */
    private static void permutingC(final double[][] tallMatrix, 
                                   boolean[][] used,
                                   int level,
                                   double sum,
                                   double[] maxTotalSS)
    {
        if (level==tallMatrix[0].length)
        {
            if (sum>maxTotalSS[0])
                maxTotalSS[0]=sum;
            
            return;
        }
        
        for (int i=0; i<tallMatrix.length; ++i)
        {
            if (!used[i][level])
            {
                //=== Mark the related row elements to the right
                for (int col=level; col<tallMatrix[0].length; ++col)
                    used[i][col]=true;
                
                //=== Set sum and level
                sum+=tallMatrix[i][level];
                level++;
                
                //=== Select next SS
                permutingC(tallMatrix, used, level, sum, maxTotalSS);
                
                //=== Reset level and sum
                level--;
                sum-=tallMatrix[i][level];
                
                //=== Unmark the related row elements to the right
                for (int col=level; col<tallMatrix[0].length; ++col)
                    used[i][col]=false;
            }           
        }
  
    }
    
    
    
    /* Generate Mask string. For example: "101100" means
                 * the 0th, second and 3rd column of SS are dumped
                 * (counting the mask from LEFT to RIGHT)
                 */
    private static void permutingRowByRow(int pos, 
                                          MaskInfo maskInfo, 
                                          double[][] SS, 
                                          boolean[][] used, 
                                          double[] maxTotalSS)
    {
        maskInfo.countOne++;
        for (; pos<maskInfo.totalBits; ++pos)
        {
            ////maskInfo.base.replace(pos, pos+1, "1");
            // Mark this column of used[][]
            for (int row=0; row<used.length; ++row)
                used[row][pos]=true;
            
            if (maskInfo.countOne == maskInfo.bitsOfOne)
            {
                //PAY ATTENTION
                
                int level=0;
                double sum=0.0;
                permutingR(SS, used, level, sum, maxTotalSS);
                //maskInfo.combinationIndex++;
            }
            else
            {
                permutingRowByRow(pos+1, maskInfo, SS, used, maxTotalSS);
            }
            
            ////maskInfo.base.replace(pos, pos+1, "0");
            // Ummark this column of used[][]
            for (int row=0; row<used.length; ++row)
                used[row][pos]=false;
        }
        
        maskInfo.countOne--;       
    }
   
    
    private static int posRecord=-1;
    
    /////////////////////////////////////////////////////////////
    private static void permutingRowByRow(MaskInfo maskInfo, 
                                          double[][] SS, 
                                          boolean[][] used, 
                                          double[] maxTotalSS)
    {
        int pos=posRecord+1;
        
        maskInfo.countOne++;
        for (; pos<maskInfo.totalBits; ++pos)
        {
            ////maskInfo.base.replace(pos, pos+1, "1");
            // Mark this column of used[][]
            for (int row=0; row<used.length; ++row)
                used[row][pos]=true;
            
            if (maskInfo.countOne == maskInfo.bitsOfOne)
            {
                //PAY ATTENTION
                
                int level=0;
                double sum=0.0;
                permutingR(SS, used, level, sum, maxTotalSS);
                //maskInfo.combinationIndex++;
            }
            else
            {
                posRecord=pos;
                permutingRowByRow(maskInfo, SS, used, maxTotalSS);
            }
            
            ////maskInfo.base.replace(pos, pos+1, "0");
            // Ummark this column of used[][]
            for (int row=0; row<used.length; ++row)
                used[row][pos]=false;
        }
        
        maskInfo.countOne--;       
    }
    
    /////////////////////////////////////////////////////////////
    
    
    
    
    
    private static void permutingColByCol(int pos, 
                                          MaskInfo maskInfo, 
                                          double[][] SS, 
                                          boolean[][] used, 
                                          double[] maxTotalSS)
    {
        maskInfo.countOne++;
        for (; pos<maskInfo.totalBits; ++pos)
        {
            ////maskInfo.base.replace(pos, pos+1, "1");
            // Mark this row of used[][]
            for (int col=0; col<used[0].length; ++col)
                used[pos][col]=true;
            
            if (maskInfo.countOne == maskInfo.bitsOfOne)
            {
                //PAY ATTENTION
                
                int level=0;
                double sum=0.0;
                permutingC(SS, used, level, sum, maxTotalSS);
                //maskInfo.combinationIndex++;
            }
            else
            {
                permutingColByCol(pos+1, maskInfo, SS, used, maxTotalSS);
            }
            
            ////maskInfo.base.replace(pos, pos+1, "0");
            // Unmark this row of used[][]
            for (int col=0; col<used[0].length; ++col)
                used[pos][col]=false;
        }
        
        maskInfo.countOne--;       
    }
    
    
    
    private static int lettersOf(String str)
    {
        int letters=0;
        
        /*
        // Include all the characters except SPACE
        String[] substrs=str.split(" ");     
        for (String substr: substrs)
            letters+=substr.length();
        */
        
        // Include ONLY letters
        for (int i=0; i<str.length(); ++i)
        {
            if ( ('A'<=str.charAt(i) && str.charAt(i)<='Z') ||
                 ('a'<=str.charAt(i) && str.charAt(i)<='z') )
            {
                letters++;
            }       
        }
        
        return letters;
    }
    
    private static int vowelsOf(String str)
    {
        int vowels=0;        
        for (int i=0; i<str.length(); ++i)
        {
            if ( str.charAt(i) == 'a' || str.charAt(i) == 'A' ||
                 str.charAt(i) == 'e' || str.charAt(i) == 'E' ||
                 str.charAt(i) == 'i' || str.charAt(i) == 'I' ||
                 str.charAt(i) == 'o' || str.charAt(i) == 'O' ||
                 str.charAt(i) == 'u' || str.charAt(i) == 'U' ||
                 str.charAt(i) == 'y' )//|| str.charAt(i) == 'Y' )
            {
                vowels++;
            }         
        }
        
        return vowels;
    }
        
    private static int consonantsOf(String str)
    {
        int consonants=0;        
        for (int i=0; i<str.length(); ++i)
        {
            if ( ('A'<=str.charAt(i) && str.charAt(i)<='Z') ||
                 ('a'<=str.charAt(i) && str.charAt(i)<='z') )
            {
                if ( str.charAt(i) == 'a' || str.charAt(i) == 'A' ||
                     str.charAt(i) == 'e' || str.charAt(i) == 'E' ||
                     str.charAt(i) == 'i' || str.charAt(i) == 'I' ||
                     str.charAt(i) == 'o' || str.charAt(i) == 'O' ||
                     str.charAt(i) == 'u' || str.charAt(i) == 'U' ||
                     str.charAt(i) == 'y' )//|| str.charAt(i) == 'Y' )
                {}
                else
                    consonants++;
            }
        }
        
        return consonants;     
    }
    
    private static int gcdOf(int num1, int num2)
    {
        if (num1<=1 || num2<=1) return 1;
        
        // Compute the great common divisor
        // Euclidean algorithm
        int minValue=num1, difference=num2;
        while (minValue != difference)
        {
            num1=minValue;
            num2=difference;
            minValue=Math.min(num1, num2);
            difference=Math.abs(num1 - num2);
        }
        
        return minValue; // Here, minValue==difference
    }
    
    
    /*
     * Static inner class
     * Store informations for one mask string
     */
    private static class MaskInfo
    {
        private int totalBits;
        private int bitsOfOne;
        private int countOne=0;
        //private int combinationIndex=0;
        //private StringBuffer base= new StringBuffer();
        
        MaskInfo(int m, int n)
        {
            totalBits=m;
            bitsOfOne=n;
            /*
            for (int i=0; i<m; ++i)
                base.append('0');
            */
        }          
    }
 
    
}
