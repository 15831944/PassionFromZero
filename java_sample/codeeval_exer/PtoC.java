/*
 * *****Solution of "Discount Offers"*****
 * Problem Description: https://www.codeeval.com/public_sc/48/
 * @author: Haiyue (Bruce) Li
 * @date: 09/20/2013
 * @email: jkzuoluo19@gmail.com, haiyuel@andrew.cmu.edu
 * 
 * Copyright (c) 2013, Haiyue Li. All rights reserved.
 */

import static java.lang.System.*;
import java.lang.Math;
import java.io.File;
import java.util.Scanner;
import java.util.Vector;

import java.io.FileNotFoundException;


public class PtoC //Product to Customer
{ 
    public static void main(String[] args) throws FileNotFoundException
    {      
        // Calcute Maximum SS for each case
        PtoC p2c= new PtoC();
        Vector<Double> output=p2c.calcMaximumSS(new File(args[0]));
              
        // Print in required format
        for (int i=0; i<output.size(); ++i)
            out.printf("%.2f\n", output.elementAt(i));
    }
    
    public Vector<Double> calcMaximumSS(File file) throws FileNotFoundException
    {
        Scanner scanner = new Scanner(file);
        
        //--- Read all the cases line by line
        Vector<String> cases = new Vector<String>(10, 5); 
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
            out.println(cases.elementAt(i));
        */
        
        //--- For each and every case, calculate the suitability score
        Vector<Double> output= new Vector<Double>(cases.size());
        
        for (int i=0; i<cases.size(); ++i)
        {
            //=== Split each case by ';' and ',', to build customers and products
            String[] caseInfo = cases.elementAt(i).split(";");
            String[] customers = caseInfo[0].split(",");
            String[] products = caseInfo[1].split(",");
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
            
            //=== Enumerate all the permutation
            Vector<Double> totalSSPermutation= permuteSS(SS);
            /*
            //DEBUG
            out.println("permutation:"+totalSSPermutation.size());
            for (int index=0; index<totalSSPermutation.size(); ++index)
                out.print(totalSSPermutation.elementAt(index)+",\t");
            
            out.println("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
            */
            
            //=== Calculate the maximum among all the permutation
            double maxTotalSS=0.0;
            for (int index=0; index<totalSSPermutation.size(); ++index)
            {
                if (totalSSPermutation.elementAt(index) > maxTotalSS)
                    maxTotalSS=totalSSPermutation.elementAt(index);           
            }
            
            output.add(maxTotalSS);
        }
        
        return output;     
    }
    
    private static double[][] calcSuitabilityScore(final String[] customers, final String[] products)
    {
        double[][] SS= new double[products.length][customers.length];    
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
        int[] productsLetters= new int[products.length];
        for (int i=0; i<products.length; ++i)
            productsLetters[i]=lettersOf(products[i]);
            
        int[] customersLetters= new int[customers.length];
        for (int i=0; i<customers.length; ++i)
            customersLetters[i]=lettersOf(customers[i]);
        
        //=== Algorithm stated in the problem online
        for (int i=0; i<products.length; ++i)
        {
            if ((productsLetters[i] % 2)==0)  //even
            {
                for (int j=0; j<customers.length; ++j)
                {
                    SS[i][j] = vowelsOf(customers[j])*1.5;
                    if ( gcdOf(productsLetters[i], customersLetters[j])>1 )
                        SS[i][j]*=1.5;
                }
            }
            else //odd
            {
                for (int j=0; j<customers.length; ++j)
                {
                    SS[i][j] = consonantsOf(customers[j]);
                    if ( gcdOf(productsLetters[i], customersLetters[j])>1 )
                        SS[i][j]*=1.5;
                }
            }       
        }
             
        return SS;
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
            if ( str.charAt(i) == 'a' ||
                 str.charAt(i) == 'e' ||
                 str.charAt(i) == 'i' ||
                 str.charAt(i) == 'o' ||
                 str.charAt(i) == 'u' ||
                 str.charAt(i) == 'y' ||
                 str.charAt(i) == 'A' ||
                 str.charAt(i) == 'E' ||
                 str.charAt(i) == 'I' ||
                 str.charAt(i) == 'O' ||
                 str.charAt(i) == 'U' )//||
                 //str.charAt(i) == 'Y' )
            {
                vowels++;
            }         
        }
        
        return vowels;
    }
        
    private static int consonantsOf(String str)
    {
        return lettersOf(str)-vowelsOf(str);     
    }
    
    private static int gcdOf(int num1, int num2)
    {
        if (num1<1 || num2<1) return 1;
        
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
    
    private static int factorialOf(int num)
    {    
        if (num==1 || num==0) return 1;
     
        return num*factorialOf(num-1);
    }
      
    private static double[][] transposeMatrix(double[][] matrix)
    {
        double[][] newMatrix= new double[matrix[0].length][matrix.length];
        
        for (int row=0; row<newMatrix.length; ++row)
            for (int col=0; col<newMatrix[0].length; ++col)
                newMatrix[row][col]=matrix[col][row];
                     
        return newMatrix;
    }
    
    private static Vector<Double> permuteSS(double[][] SS)
    {   
        //=== Check whether the SS needs to be transposed
        if (SS.length > SS[0].length) //products>customers
            SS=transposeMatrix(SS);
        
        //~~~~~~~~~ Now, SS[0].length >= SS.length ~~~~~~~~~//
        //=== Total allocation situations
        int situations = factorialOf(SS[0].length) / factorialOf(SS[0].length-SS.length);
        Vector<Double> allSituationsOfTotalSS= new Vector<Double>(situations);
        
        //=== Record whether or not the element in SS is used
        boolean[][] used= new boolean[SS.length][SS[0].length];
        
        //=== Record which row is being chosen; record current sum of chosen SS
        int level=0;
        double sum=0.0;
        
        //=== Perform permuting
        if (SS[0].length==SS.length)// Num of customers == Num of products
        {
            permuting(SS, used, level, sum, allSituationsOfTotalSS);
        }
        else
        {
            /* Generate Mask string. For example: "101100" means
             * the 0th, second and 3rd column of SS are dumped
             * (counting the mask from LEFT to RIGHT)
             */
            String[] masks=combinationMask(SS[0].length, SS[0].length-SS.length);
            /*
            //DEBUG
            out.println("Num of customers != Num of products, create masks");
            out.println("Situation of masks="+masks.length);
            */
            
            for (int i=0; i<masks.length ;++i)
            {
                // Set used[][] according to masks[i] 
                setUsed(resetBooleanArray2D(used), masks[i]);
                /*
                //DEBUG
                out.println("mask["+i+"]: "+masks[i]);
                for (int row=0; row<used.length; ++row)
                {
                    for (int col=0; col<used[0].length; ++col)
                        out.print(used[row][col]+",\t");
                    
                    out.println();
                }
                */
                
                // Reset level and sum
                level=0;
                sum=0.0;
                
                permuting(SS, used, level, sum, allSituationsOfTotalSS);
            }    
        }
        
        return allSituationsOfTotalSS;
    }
    
    private static boolean[][] resetBooleanArray2D(boolean[][] array2D)
    {
        for (int row=0; row<array2D.length; ++row)
            for (int col=0; col<array2D[0].length; ++col)
                array2D[row][col]=false;
        
        return array2D;
    }
       
    private static void setUsed(boolean[][] used, String mask)
    {
        for (int i=0; i<mask.length(); ++i)
        {
            if (mask.charAt(i)=='1')
            {
                // Mark this column of used[][]
                for (int row=0; row<used.length; ++row)
                    used[row][i]=true;
            }         
        }   
    }
    
    /**
     * Generate all the situations of combined SS
     * @param flatMatrix: 'flat' means matrix[0].length >= matrix.length
     * @param used: whether this SS has been chosen
     * @param level: which row of the SS[][] is being handled
     * @param sum: sum of SSs that have been chosen
     * @param allSituationsOfTotalSS: the output 
     */
    private static void permuting(final double[][] flatMatrix, 
                                  boolean[][] used,
                                  int level,
                                  double sum,
                                  Vector<Double> allSituationsOfTotalSS)
    {
        if (level==flatMatrix.length)
        {
            allSituationsOfTotalSS.add(sum);
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
                permuting(flatMatrix, used, level, sum, allSituationsOfTotalSS);
                
                //=== Reset level and sum
                level--;
                sum-=flatMatrix[level][i];
                
                //=== Unmark the related column elements below
                for (int row=level; row<flatMatrix.length; ++row)
                    used[row][i]=false;
            }           
        }
  
    }
    
    
    private static class MaskInfo
    {
        private int totalBits;
        private int bitsOfOne;
        private int countOne=0;
        private int combinationIndex=0;
        private StringBuffer base= new StringBuffer();
        
        MaskInfo(int m, int n)
        {
            totalBits=m;
            bitsOfOne=n;
            for (int i=0; i<m; ++i)
                base.append('0');
        }          
    }
    
    /**
     * Generate masks corresponding to all the combinations of '0' and '1'
     * @param m: total bits
     * @param n: bits of '1'
     * @return String[] masks containing all the combination
     */
    private static String[] combinationMask(final int m, final int n)
    {
        //=== Combination(m, n)
        int combinations= factorialOf(m) / factorialOf(n) / factorialOf(m-n);
            
        //=== Create allSituationsOfTotalSS
        String[] masks= new String[combinations];
        
        //=== Set mask bit by bit
        MaskInfo maskInfo=new MaskInfo(m, n);
        bitSet(maskInfo, masks);
        
        //DEBUG
        out.println("combinations="+combinations);
        for (int i=0; i<combinations; ++i)
            out.println(masks[i]);
        
        
        return masks;
    }
     
    private static int posRecord=-1;
    
    private static void bitSet(MaskInfo maskInfo, String[] masks)
    {
        int pos=posRecord+1;
        
        maskInfo.countOne++;
        for (; pos<maskInfo.totalBits; ++pos)
        {
            maskInfo.base.replace(pos, pos+1, "1");
            if (maskInfo.countOne == maskInfo.bitsOfOne)
            {
                //out.println(maskInfo.base+", maskInfo.combinationIndex: "+maskInfo.combinationIndex);
                masks[maskInfo.combinationIndex]=new String(maskInfo.base);
                maskInfo.combinationIndex++;
            }
            else
            {
                posRecord=pos;
                bitSet(maskInfo, masks);
            }
            
            maskInfo.base.replace(pos, pos+1, "0");
        }
        
        maskInfo.countOne--;       
    }
    
    private static void bitSet(int pos, MaskInfo maskInfo, String[] masks)
    {
        
        maskInfo.countOne++;
        for (; pos<maskInfo.totalBits; ++pos)
        {
            maskInfo.base.replace(pos, pos+1, "1");
            if (maskInfo.countOne == maskInfo.bitsOfOne)
            {
                out.println(maskInfo.base+", maskInfo.combinationIndex: "+maskInfo.combinationIndex);
                masks[maskInfo.combinationIndex]=new String(maskInfo.base);
                maskInfo.combinationIndex++;
            }
            else
            {
                bitSet(pos+1, maskInfo, masks);
            }
            
            maskInfo.base.replace(pos, pos+1, "0");
        }
        
        maskInfo.countOne--;       
    }
    
    
}
