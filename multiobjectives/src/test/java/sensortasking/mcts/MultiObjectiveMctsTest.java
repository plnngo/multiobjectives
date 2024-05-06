package sensortasking.mcts;

import java.io.File;
import java.util.List;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.time.AbsoluteDate;

public class MultiObjectiveMctsTest {

    /** Tree structure stored in a root node. */
    Node root;

    @Before
    public void init() {
        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        // Build up test decision tree
        root = new DecisionNode(28, 4, null, null, null, new AbsoluteDate(), null);
        root.setId(0);
        
        ChanceNode child1 = new ChanceNode(0, 17, 2, null, null, root);
        ChanceNode child2 = new ChanceNode(0, 8, 2, null, null, root);
        child1.setId(1);
        child2.setId(2);
        root.setChild(child1);
        root.setChild(child2);

        DecisionNode grandchild1 = new DecisionNode(10, 1, null, null, null, new AbsoluteDate(), null);
        DecisionNode grandchild2 = new DecisionNode(7, 1, null, null, null, new AbsoluteDate(), null);
        DecisionNode grandchild3 = new DecisionNode(3, 1, null, null, null, new AbsoluteDate(), null);
        DecisionNode grandchild4 = new DecisionNode(5, 1, null, null, null, new AbsoluteDate(), null);
        grandchild1.setId(3);
        grandchild2.setId(4);
        grandchild3.setId(5);
        grandchild4.setId(6);
        child1.setChild(grandchild1);
        child1.setChild(grandchild2);
        child2.setChild(grandchild3);
        child2.setChild(grandchild4);

        ChanceNode ggchild1 = new ChanceNode(0, 10, 1, null, null, grandchild1);
        ChanceNode ggchild2 = new ChanceNode(0, 7, 1, null, null, grandchild2);
        ggchild1.setId(7);
        ggchild2.setId(8);
        grandchild1.setChild(ggchild1);
        grandchild2.setChild(ggchild2);

        DecisionNode gggchild1 = new DecisionNode(10, 1, null, null, null, new AbsoluteDate(), null);
        DecisionNode gggchild2 = new DecisionNode(7, 1, null, null, null, new AbsoluteDate(), null);
        gggchild1.setId(9);
        gggchild2.setId(10);
        ggchild1.setChild(gggchild1);
        ggchild2.setChild(gggchild2);
    }   

    @Test
    public void testSelectChild() {

        Node parent = new Node();
        parent.setUtility(50);
        parent.setNumVisits(9);

        int[] numVisits = new int[]{3, 6, 1, 7, 8, 3, 2, 1, 1, 9};
        for(int i=0; i<numVisits.length; i++){
            double utility = i * 10;
            Node child = new Node();
            child.setUtility(utility);
            child.setNumVisits(numVisits[i]);

            parent.setChild(child);
        }

        Node actuallySelected = MultiObjectiveMcts.selectChild(parent);

        // Last child is expected to reveal largest UCB because of its large utility value
        Node expectedlySelected = parent.getChildren().get(numVisits.length - 1);
        Assert.assertEquals(expectedlySelected, actuallySelected);

    }

    @Test
    public void testSelect() {
        // Compare
        List<Node> actual = MultiObjectiveMcts.select(root);
        long[] expectedIds = new long[]{0, 1, 3, 7, 9};
        for(int i=0; i<expectedIds.length; i++) {
            Assert.assertEquals(expectedIds[i], actual.get(i).getId());
        }
    }

    @Test
    public void testBackpropagate() {

        // Set up
        long[] expectedIds = new long[]{0, 1, 3, 7, 9};
        List<Node> selected = MultiObjectiveMcts.select(root);
        MultiObjectiveMcts mcts = new MultiObjectiveMcts(root, null, null, null);

        // Add fakely simulated nodes
        DecisionNode termination = new DecisionNode(1, 1, null, null, null, null, null);

        mcts.backpropagate(selected, termination);
        Node updatedRoot = mcts.getInitial();
        List<Node> updatedEpisode = MultiObjectiveMcts.select(updatedRoot);

        // Updated tree should result in the same nodes selected using the UCB condition
        for(int i=0; i<expectedIds.length; i++) {
            Assert.assertEquals(expectedIds[i], updatedEpisode.get(i).getId());
        }

        // Compare root node
        Assert.assertEquals(5, updatedEpisode.get(0).getNumVisits());
        Assert.assertEquals(29, updatedEpisode.get(0).getUtility(),1e-16);

        // Compare next descendant
        Assert.assertEquals(3, updatedEpisode.get(1).getNumVisits());
        Assert.assertEquals(18, updatedEpisode.get(1).getUtility(),1e-16);

        // Compare the rest of the episode
        for(int i=2; i<updatedEpisode.size(); i++) {
            Assert.assertEquals(2, updatedEpisode.get(i).getNumVisits());
            Assert.assertEquals(11, updatedEpisode.get(i).getUtility(),1e-16);
        }
    }
}
