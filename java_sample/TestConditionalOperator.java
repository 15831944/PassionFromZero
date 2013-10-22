// test ? :
import javax.swing.JOptionPane;

public class TestConditionalOperator
{


    public static void main(String[] args)
    {
        int score;
        score = Integer.parseInt(JOptionPane.showInputDialog("Please input score:"));
        String result = (score>=60)?"Pass":"Failed";
        System.out.println(result);
    }




}
