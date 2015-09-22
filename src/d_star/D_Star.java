package d_star;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Scanner;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * D* implementation based on Anthony Stentz's algorithm from his 1994 paper
 * Optimal and Efficient Path Planning for Partially-Known Environments
 * @author Kevin Dittmar
 */
public class D_Star
{
//    private final ArrayList<Node[]> world;
    private final ArrayList<Node> world;
    private final ArrayList<Node> path;
    private int rows;
    private int cols;
    
    private final ArrayList<Node> open_list;
    
    private Node robot;
    private Node goal;
    
    public D_Star(String file_path)
    {
        world = new ArrayList<>();
        open_list = new ArrayList<>();
        path = new ArrayList<>();
        parseFile(file_path);
    }
    
    /**
     * Parse a file for the map of the world to use.
     * Note:  This method expects a rectangular grid world.
     * @param file_path the path of the file to parse for world information.
     */
    private void parseFile(String file_path)
    {
        File file = new File(file_path);
        Scanner scanner;
        try
        {
            scanner = new Scanner(file);
        
            while (scanner.hasNextLine())
            {
                String next_line = scanner.nextLine();
                cols = next_line.length();
                for (int i = 0; i < next_line.length(); i++)
                {
                    char type = next_line.charAt(i);
                    Node node = new Node(rows, i, type);
                    if (type == Node.START)
                    {
                        robot = node;
                    }
                    else if (type == Node.GOAL)
                    {
                        goal = node;
                    }
                    world.add(node);
                }
                rows++;
            }
        }
        catch (FileNotFoundException ex)
        {
            Logger.getLogger(D_Star.class.getName()).log(Level.SEVERE,null,ex);
        }
        System.out.println("DONE PARSING WORLD");
    }
    
    /**
     * Process the state on the open list with the smallest k value.
     * @return the smallest k value on the open list.
     */
    private float processState()
    {
        Node x = minState();
        if (x == null)
        {
            return -1;
        }
        
        /* Testing:  Make sure that the current min k is the same as the
         * proposed min state's k value.
         */
        if (!equals(x.getK(), getKMin()))
        {
            System.err.println("Error: lists out of sync.");
            System.exit(1);
        }
        delete(x);
        ArrayList<Node> neighbors = getNeighbors(x);
        //RAISE state
        if (x.getK() < x.getH())
        {
            for (Node y : neighbors)
            {
                if (y.getH() < x.getK() &&
                    x.getH() > y.getH() + cost(y,x))
                {
                    x.setBackpointer(y);
                    x.setH(y.getH() + cost(y,x));
                }
            }
        }
        
        //LOWER state
        if (equals(x.getK(), x.getH()))
        {
            for (Node y : neighbors)
            {
                if (y.getState() == Node.NEW ||
                    (x == y.getBackpointer() && 
                     !equals(y.getH(), x.getH() + cost(x,y))) ||
                    (x != y.getBackpointer() &&
                     y.getH() > x.getH() + cost(x,y)))
                {
                    y.setBackpointer(x);
                    insert(y, x.getH() + cost(x,y));
                }
            }
        }
        else
        {
            for (Node y : neighbors)
            {
                if (y.getState() == Node.NEW ||
                        (y.getBackpointer() == x &&
                         !equals(y.getH(), x.getH() + cost(x,y)))
                    )
                {
                    y.setBackpointer(x);
                    insert(y, x.getH() + cost(x,y));
                }
                else
                {
                    if (y.getBackpointer() != x &&
                        y.getH() > x.getH() + cost(x,y))
                    {
                        insert(x, x.getH());
                    }
                    else
                    {
                        if (y.getBackpointer() != x && 
                            x.getH() > y.getH() + cost(y,x) &&
                            y.getState() == Node.CLOSED &&
                            y.getH() > x.getK())
                        {
                            insert(y,y.getH());
                        }
                    }
                }
            }
        }
        return getKMin();
    }
    
    /**
     * Iterate over the open list to find the open state with the smallest
     * k value.
     * @return the open state with the smallest k.
     */
    private Node minState()
    {
        Node min_state = null;
        if (open_list.size() > 0)
        {
            min_state = open_list.get(0);
        }
        return min_state;
    }
    
    /**
     * Get the minimum k value of all states on
     * the open list.
     * @return the minimum k value.
     */
    private float getKMin()
    {
        float k = -1.0f;
        if (open_list.size() > 0)
        {
            k = open_list.get(0).getK();
        }
        return k;
    }
    
    private void delete(Node x)
    {
        x.setState(Node.CLOSED);
        open_list.remove(x);
    }

    /**
     * Access the single-dimensional ArrayList as if it was
     * a grid with rows and columns.
     * @param row is the row of the node to access (0 <= row < rows)
     * @param col is the column of the node to access (0 <= rows < cols)
     * @return the Node at the requested row and column, or null if out of
     * bounds.
     */
    private Node getNode(int row, int col)
    {
        if (row >= 0 && row < rows && col >= 0 && col < cols)
        {
            return world.get(row * cols + col);
        }
        else
        {
            return null;
        }
    }
    
    
    /**
     * Update Node x's k value, put it on the open list, and give it the new
     * h value.
     * @param x is the Node to be added to the open list.
     * @param h_new is the h value to be given to Node x.
     */
    private void insert(Node x, float h_new)    
    {
        if (x.getState() == Node.NEW)
        {
            x.setK(h_new);
        }
        else if (x.getState() == Node.OPEN)
        {
            x.setK(Math.min(x.getK(), h_new));
        }
        else if (x.getState() == Node.CLOSED)
        {
            x.setK(Math.min(x.getH(), h_new));
        }
        x.setH(h_new);
        x.setState(Node.OPEN);
        
        //Prevent a node from being added to the open list multiple times.
        if (!open_list.contains(x))
        {
            open_list.add(x);
        }
        Collections.sort(open_list);
    }
    
    /**Gets all neighbors of the node x.
     * @param x is the node in question.
     * @return all valid nodes adjacent to x.
     */
    private ArrayList<Node> getNeighbors(Node x)
    {
        ArrayList<Node> neighbors = new ArrayList<>();
        int row = x.getRow();
        int col = x.getCol();
        
        for (int row_mod = -1; row_mod < 2; row_mod++)
        {
            for (int col_mod = -1; col_mod < 2; col_mod++)
            {
                Node neighbor = getNode(row + row_mod, col + col_mod);
                
                /* Don't add any null nodes (outside of the node table) or
                 * the node itself to the neighbor list.
                 */
                if (neighbor != null && neighbor != x)
                {
                    neighbors.add(neighbor);
                }
            }
        }
        return neighbors;
    }
    
    /**
     * Find the cost to travel from Node y to Node x.
     * @param y is the neighbor node
     * @param x is the perspective node
     * @return the cost to travel from y to x.
     * Pre:  Nodes y and x must be neighboring nodes that aren't the same node.
     */
    float cost(Node y, Node x)
    {
        if (x.getType() == Node.BLOCKED ||
            y.getType() == Node.BLOCKED)
        {
            return Node.INFINITY;
        }
        //The nodes are vertically or horizontally adjacent
        else if (x.getRow() - y.getRow() == 0 ||
                 x.getCol() - y.getCol() == 0)
        {
            return 1.0f;
        }
        else
        {
            return 1.4f;
        }
    }
    
    /**
     * Modify the cost Y (the node that is now blocked) and put X back
     * on the open list because it turned out that Y was not a clear path.
     * @param x is the Node where the robot is.
     * @param y is the blocked node that the robot thought was clear.
     * @return the minimum k value on the open list.
     */
    private float modifyCost(Node x, Node y)
    {
        //We know that the state is blocked now, so make it blocked.
        y.blockNode();
        
        /* Put y back on the open list with an infinte cost since it's now
         * blocked.
         */
        insert(y, Node.INFINITY);
        
        if (x.getState() == Node.CLOSED)
        {
            //X just got its path blocked, so put it on the OPEN list.
            insert(x, x.getH());
        }
        return getKMin();
    }
    
    /**
     * Test float equality.
     * @param x is the first float operand
     * @param y is the second float operand
     * @return true if the floats are equal within epsilon = .001 or both are
     * infinity, false otherwise.
     */
    private boolean equals(float x, float y)
    {
        return (x >= Node.INFINITY && y >= Node.INFINITY) ||
               (Math.abs(x - y) < .0001);
    }
    
    /**
     * Do the D* pathing algorithm for this world.
     */
    public void execute()
    {
        try
        {
            //Allocate a file for output.
            File file = new File("output.txt");
            FileWriter writer = new FileWriter(file);
            
            //Add the goal to the open list.
            insert(goal, 0.0f);
            
            /* The robot's path has to include his start state to say where it
             * started.
             */
            path.add(robot);
            
            float k_min = 0.0f;
            
            //Add starting world to the output file.
            writer.append(worldToString(robot));
            
            //Process states until the robot's current state is closed.
            while (robot.getState() != Node.CLOSED &&
                   k_min >= 0)
            {
                k_min = processState();
                
                //Add the newly processed world to the output file.
                writer.append(worldToString(robot));
            }
            
            //No path exists and the open list is empty.
            if (k_min < 0)
            {
                writer.flush();
                writer.close();
                System.err.println("No possible path.");
                System.exit(1);
            }
            
            /* Processing is done; follow backpointers to the goal node.
            */
            while (robot.getBackpointer() != goal)
            {
                /* The robot isn't trying to move onto a blocked node, so add
                * the node to the path.
                */
                if (robot.getBackpointer().getType() != Node.BLOCKED &&
                robot.getBackpointer().getType() != Node.UNKNOWN_BLOCKED)
                {
                    //The robot should be in the backpointer state.
                    robot = robot.getBackpointer();
                    
                    //Add the node that the robot now occupies to the path.
                    path.add(robot);
                    
                    //Add the updated world to the output file.
                    writer.append(worldToString(robot));
                }
                //We need to find a new path.
                else
                {
                    Node x = robot;
                    Node y = robot.getBackpointer();
                    k_min = modifyCost(x, y);
                    /* We can continue following the path when back-propagation
                     * is finished enough to close the robot's state again.
                     */
                    while (!(k_min >= y.getH()) && 
                           robot.getState() != Node.CLOSED)
                    {
                        k_min = processState();
                        
                        //Add the newly processed world to the output file.
                        writer.append(worldToString(robot));
                    }
                }
            }
            
            //Print the path up to the goal.
            for (Node node : path)
            {
                System.out.println(node.getName() + " -> ");
            }
            /* The backpointer points to the goal, so print its name to
             * complete the path.
             */
            System.out.println(robot.getBackpointer().getName());
            
            /* Add the final world to the output file, flush the writer,
             * and close the writer.
             */
            writer.append(worldToString(robot.getBackpointer()));
            writer.flush();
            writer.close();
        }
        catch (IOException ex)
        {
            Logger.getLogger(D_Star.class.getName()).log(Level.SEVERE,null,ex);
        }
    }
    
    /**
     * Represent the current state of the world as a String, including the
     * current position of the robot.
     * @param robot is the Node that the robot occupies.
     * @return the String representation of the world.
     */
    private String worldToString(Node robot)
    {
        String result = "";
        for (int row = 0; row < rows; row++)
        {
            //Print Name
            for (int col = 0; col < cols; col++)
            {
                Node node = getNode(row, col);
                result += gridBlock(node.getName());
            }
            result += "\n";
            //Print h
            for (int col = 0; col < cols; col++)
            {
                Node node = getNode(row, col);
                result += gridBlock("h: " + Float.toString(node.getH()));
            }
            result += "\n";
            //Print k
            for (int col = 0; col < cols; col++)
            {
                Node node = getNode(row, col);
                result += gridBlock("k: " + Float.toString(node.getK()));
            }
            result += "\n";
            //Print b
            for (int col = 0; col < cols; col++)
            {
                Node b = getNode(row, col).getBackpointer();
                if (b != null)
                {
                    result += gridBlock("b: " + b.getName());
                }
                else
                {
                    result += gridBlock("b:");
                }
            }
            result += "\n";
             //Print state
            for (int col = 0; col < cols; col++)
            {
                Node node = getNode(row, col);
                int state = node.getState();
                String state_name = "";
                if (state == Node.NEW)
                {
                    state_name = "NEW";
                }
                else if (state == Node.OPEN)
                {
                    state_name = "OPEN";
                }
                else if (state == Node.CLOSED)
                {
                    state_name = "CLOSED";
                }
                result += gridBlock(state_name);
            }
            result += "\n";
            //Print type
            for (int col = 0; col < cols; col++)
            {
                Node node = getNode(row, col);
                result += gridBlock(Character.toString(node.getType()));
            }
            result += "\n";
            //Print "Robot" if it's currently here
            boolean line_has_robot = false;
            for (int col = 0; col < cols; col++)
            {
                Node node = getNode(row, col);
                if (node == robot)
                {
                    line_has_robot = true;
                    result += gridBlock("ROBOT");
                }
                else
                {
                    result += gridBlock("");
                }
            }
            
            if (line_has_robot)
            {
                result += "\n";
            }
            result += "\n\n";
        }
        return result + "**************************************************" +
                        "**************************************************" +
                        "\n\n";
    }
    
    /**
     * Turn the given String into a String of standardized length.  If the
     * String isn't 15 characters long, pad the end with spaces.
     * @param string is the String to format.
     * @return a String that is exactly 15 characters long.
     */
    private String gridBlock(String string)
    {
        int length = 15 - string.length();
        String result = string;
        for (int i = 0; i < length; i++)
        {
            result += " ";
        }
        return result;
    }
    
    /**
     * @param args the command line arguments
     */
    public static void main(String[] args)
    {
        D_Star d_star = new D_Star("map3.txt");
        d_star.execute();
    }
}
