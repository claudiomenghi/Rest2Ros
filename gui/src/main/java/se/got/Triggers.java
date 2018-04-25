package se.got;

import java.util.function.BinaryOperator;

import se.got.ltl.LTLConjunction;
import se.got.ltl.LTLEventually;
import se.got.ltl.LTLFormula;
import se.got.ltl.LTLIGlobally;
import se.got.ltl.LTLIImplies;
import se.got.ltl.LTLIUntil;
import se.got.ltl.LTLNext;

public enum Triggers {

	WAIT {

		@Override
		public String toString() {
			return "Wait";
		}

		@Override
		public String getDescription() {
			return "Applies when inaction is desired till a stimulus occurs. Wait is also encountered in literature as stop.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\", where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space.";
		}

		@Override
		public String getExamples() {
			return "Given a condition c, and the areas l1, l2, l3, and l4, a wait condition may specify that the robot remains in room l1 until a condition is satisfied. If the a robot visits the areas l1, l3, l2, l1, and l4, and c occurs when the robot is in location l2 the mission is not satisfied. Vice versa, the mission is satisfied if a robot visits the areas l1, l1, l2, l1, and l4, and c occurs in the second occurrence of location l1.";
		}

		@Override
		public String getRelationships() {
			return "The LTL formula used to encode Wait Pattern is a simple UNTIL formula that forces the robot to remain in l1 until condition c is met.";
		}

		@Override
		public String getOccurrences() {
			return "";
		}

		@Override
		public LTLFormula getMission(LTLFormula f1, LTLFormula f2) {
			return new LTLIUntil(f1, f2);
		}
	},
	INSTANTANEOUS_REACTION {

		@Override
		public String toString() {
			return "Instantaneous Reaction";
		}
		@Override
		public String getDescription() {
			return "Applies when the occurrence of a stimulus instantaneously triggers a counteraction.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\", where in indicates that the robot r is inside location l, and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\", where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space.";
		}

		@Override
		public String getExamples() {
			return "For example, given a condition c, and the areas l1, l2, l3, and l4 an instantaneous reaction specifies that when c occurs the robot must start moving toward area l1, and l1 is finally reached. If the a robot visits the areas l1, l3, l2, l2, l4, and c occurs when the robot is in location l2, the mission is not satisfied. If the a robot visits the areas l1, l3, l2, l1, l4, and c occurs when the robot is in location l2 the mission is satisfied. The mission is also satisfied if a robot visits the areas l1, l3, l2, l1, and l4, and c occurs when the robot is in location l1.";
		}

		@Override
		public String getRelationships() {
			return "	The LTL formula used to encode Instantaneous Reaction Pattern corresponds with the LTL formula used to encode the Response Pattern by Dwyer.";
		}

		@Override
		public String getOccurrences() {
			return "An example of usage of instantaneous reaction is used by Fainekos et al. to specify the mission requirement: ``If you see Mika, go to room P\".";
		}

		@Override
		public LTLFormula getMission(LTLFormula f1, LTLFormula f2) {
			return new LTLIGlobally(new LTLIImplies(f1, new LTLEventually(f2)));
		}
	},
	DELAYED_REACTION {
		@Override
		public String toString() {
			return "Delayed Reaction";
		}

		@Override
		public String getDescription() {
			return "Applies when the occurrence of a stimulus triggers a counteraction some time later.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\", where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space.";
		}

		@Override
		public String getExamples() {
			return "Given a condition c, and the areas l1, l2, l3, and l4 an instantaneous reaction specifies that when c occurs the robot must start moving toward area l1 and l1 is finally reached. If the a robot visits the areas l1, l3, l2, l1, and l4, and c occurs when the robot is in location l2 the mission is satisfied. Vice versa, the mission is not satisfied if a robot visits the areas l1, l3, l2, l1, and l4, and c occurs when the robot is in location l1.";
		}

		@Override
		public String getRelationships() {
			return "The LTL formula used to encode Delayed Reaction Pattern corresponds to the LTL formula used to encode the Response Pattern by Dwyer modified with the delay represented by the operator NEXT.";
		}

		@Override
		public String getOccurrences() {
			return "Chen et al. proposed an LTL formalization of the mission requirement ``each pickup should be immediately followed by a delivery\" that can be considered as an example of usage of the delayed reaction pattern.";
		}

		@Override
		public LTLFormula getMission(LTLFormula f1, LTLFormula f2) {
			return new LTLIGlobally(new LTLIImplies(f1, new LTLNext(new LTLEventually(f2))));
		}
	};

	public abstract LTLFormula getMission(LTLFormula f1, LTLFormula f2);

	public abstract String getDescription();

	public abstract String getVariations();

	public abstract String getExamples();

	public abstract String getRelationships();

	public abstract String getOccurrences();

	public static final BinaryOperator<LTLFormula> conjunctionOperator = (left, right) -> {

		if (left.equals(LTLFormula.TRUE)) {
			return right;
		}

		if (right.equals(LTLFormula.TRUE)) {
			return left;
		}
		return new LTLConjunction(left, right);
	};
}
