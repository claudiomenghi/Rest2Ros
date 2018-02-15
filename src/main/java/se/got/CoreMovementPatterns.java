package se.got;

import java.util.Arrays;
import java.util.List;
import java.util.function.BinaryOperator;

import se.got.ltl.LTLConjunction;
import se.got.ltl.LTLEventually;
import se.got.ltl.LTLFormula;
import se.got.ltl.LTLIGlobally;
import se.got.ltl.LTLIImplies;
import se.got.ltl.LTLINegation;
import se.got.ltl.LTLIUntil;
import se.got.ltl.LTLNext;
import se.got.ltl.atoms.LTLIPropositionalAtom;

public enum CoreMovementPatterns {

	VISIT {
		@Override
		public String toString() {
			return "Visit";
		}

		public LTLFormula getMission(String[] locations) {
			return Arrays.asList(locations).stream()
					.map(location -> (LTLFormula) new LTLEventually(new LTLIPropositionalAtom(location)))
					.reduce(LTLFormula.TRUE, conjunctionOperator);

		}

		@Override
		public String getDescription() {
			return "A robot must visit an area or a set of areas. This area can be a set of locations of a building, or a set of points of interest on a map that must be surveyed.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l, while l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\" where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space. A variation can be obtained where not all the areas in the set must be visited. In this case, it is sufficient to replace the AND operator with an OR.";
		}

		@Override
		public String getExamples() {
			return "A common usage example of the Visit pattern is a scenario in which a robot has to collect a set of items that are placed in different locations and bring them in a target destination. Visit and Avoidance patterns often go together. Avoidance patterns are used e.g. to require robots to avoid obstacles as they guard an area. Trigger patterns can also be used in combination with the Visit pattern to specify conditions upon which Visit should start or stop.";
		}

		@Override
		public String getRelationships() {
			return "The Visit pattern generalizes most of the core movement patterns that constrain how locations are visited.";
		}

		@Override
		public String getOccurrences() {
			return "Yoo et al. and Kress-Gazit et al. formulate an LTL mission specification to ensure that a set of areas are visited. In the first case, the visiting pattern is combined with the specification of past and future avoidance mission requirements. In the second case, an LTL mission specification is provided to describe the following mission requirement: the robot must go to rooms $P1$, $P2$, $P3$ and $P4$ in any order.";
		}
	},
	SEQUENCED_VISIT {
		@Override
		public String toString() {
			return "Sequenced Visit";
		}

		@Override
		public LTLFormula getMission(String[] locations) {
			List<String> loc = Arrays.asList(locations);
			LTLFormula f = new LTLEventually(new LTLIPropositionalAtom(loc.get(loc.size() - 1)));

			for (int index = loc.size() - 2; index >= 0; index--) {
				f = new LTLEventually(new LTLConjunction(new LTLIPropositionalAtom(loc.get(index)), f));
			}
			return f;

		}

		@Override
		public String getDescription() {
			return "Given a set of areas A the robot(s) should visit all the areas in the set A in sequence.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\" where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space.";
		}

		@Override
		public String getExamples() {
			return "This problem is also indicated in literature as reach, cover, or search, meaning that a robot should reach, cover or search a set areas or points, one following the other. This sequence may reflect e.g., a traversal strategy. Note that given two areas a and b that must be visited in sequence by a robot r , the sequence pattern does not forbid robot r to visit b before visiting a, but only requires the robot to visit b after and a while a must be visited.  Consider the areas l1, l2, l3, and l4 and a set of areas {l1,l2,l3} to be all covered following the sequence l1, l2, l3. If the a robot visits the areas l1, l4, l3, l1, l4, and l2 in this order, the mission is not satisfied. Vice versa, if the a robot visits the areas l1, l3, l1, l2, l4, and l3 in this order, the mission is satisfied.  Sequenced Visit and Avoidance often go together. Avoidance patterns are used to require robots to avoid obstacles as e.g. they guard an area. Triggers can also be used in combination with the Sequenced Visit pattern to specify conditions upon which Visit should start or stop.";
		}

		@Override
		public String getRelationships() {
			return "The Sequenced Visit pattern generalizes the Ordered visit patterns that further constrain how locations are visited.";
		}

		@Override
		public String getOccurrences() {
			return "Yoo et al. Kress-Gazit et al. and formulate an LTL mission specification that ensures that a set of areas are visited in sequence. Specifically, an LTL mission specification is provided for the following mission requirement: visit a set of area l1, l2 ... ln by ensuring that l1 is visited after (but not necessarily only after) area l2.";
		}
	},
	ORDERED_VISIT {
		@Override
		public String toString() {
			return "Ordered Visit";
		}

		@Override
		public LTLFormula getMission(String[] locations) {

			LTLFormula f = CoreMovementPatterns.SEQUENCED_VISIT.getMission(locations);

			List<String> loc = Arrays.asList(locations);
			LTLFormula f1 = LTLFormula.TRUE;

			
			for (int index = loc.size() - 1; index > 0; index--) {
				f1 = LTLFormula.and(f1, new LTLIUntil(new LTLINegation(new LTLIPropositionalAtom(loc.get(index))),
						new LTLIPropositionalAtom(loc.get(index-1))));
			}
			return new LTLConjunction(f, f1);
		}

		@Override
		public String getDescription() {
			return "Given a set of areas A the robot(s) should visit all the areas in the set A in order.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" which indicates that the robot r is inside location l, while l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z) where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space.";
		}

		@Override
		public String getExamples() {
			return "This problem is also indicated in literature as ordered reach, cover, or search, meaning that a robot should reach, cover or search a set areas or points following an order. Consider the areas l1, l2, l3, and l4 and a set of areas {l1,l2,l3} to be all covered following the order l1, l2, l3. If the a robot visits the areas l1, l3, l1, l2, l4 in this order, the mission is not satisfied since l3 precedes l2. Vice versa, if the a robot visits the areas l1, l4, l1, l2, l4 , and l3 in this order, the mission is satisfied.  Ordered visit can be used in combination with Avoidance and Trigger patterns. Avoidance patterns are used to force robots to avoid obstacles as e.g. they guard an area. Triggers can also be used in combination with the Visit pattern to specify conditions upon which Visit should start or stop.";
		}

		@Override
		public String getRelationships() {
			return "The Ordered visit pattern generalizes the Strict Ordered Visit pattern that further constrains how locations are visited.";
		}

		@Override
		public String getOccurrences() {
			return "An example of ordered visit can be obtained by combining the visit formulation provided by Yoo et al. and Kress-Gazit et al. , with the absence pattern proposed by Dwyer et al..";
		}
	},
	STRICT_ORDERED_VISIT {
		@Override
		public String toString() {
			return "Strict Ordered Visit";
		}

		@Override
		public LTLFormula getMission(String[] locations) {
			LTLFormula f = CoreMovementPatterns.ORDERED_VISIT.getMission(locations);

			List<String> loc = Arrays.asList(locations);
			LTLFormula f1 = LTLFormula.TRUE;

			for (int index = 0; index < loc.size()-1; index++) {
				f1 = LTLFormula.and(f1, 
						new LTLIGlobally(
								new LTLIImplies(
										new LTLIPropositionalAtom(loc.get(index)),
										new LTLNext(
												new LTLIUntil(
														new LTLINegation(new LTLIPropositionalAtom(loc.get(index))),
														new LTLIPropositionalAtom(loc.get(index+1))
														)
												)
										)
								)
					);
			}
			return new LTLConjunction(f, f1);
		}

		@Override
		public String getDescription() {
			return "Given a set of areas A the robot(s) should visit all the areas in the set A following a strict order.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\" where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space.";
		}

		@Override
		public String getExamples() {
			return "This problem is also indicated in literature as strict ordered reach, cover, or search, meaning that a robot should reach, cover or search a set areas or points following a strict order.Consider the areas l1, l2, l3, and l4 and a set of areas {l1,l2,l3} to be all covered following the strict order l1, l2, l3. If the a robot visits the areas l1, l4, l1, l2, l4, and l3 in this order, the mission is not satisfied. Vice versa, if the robot visit the areas l1, l4, l2, l4, and l3 in this order, the mission is satisfied Strict Ordered visit can be used in combination with Avoidance and Trigger patterns. Avoidance patterns are used to force robots to e.g. avoid obstacles as they guard an area. Trigger can also be used in combination with the Visit pattern to specify conditions upon which Visit should start or stop.";
		}

		@Override
		public String getRelationships() {
			return "The Strict Ordered visit pattern specializes the Ordered visit pattern by further constraining how locations are visited.";
		}

		@Override
		public String getOccurrences() {
			return "Smith et al. proposed a mission requirement that forces a robots to not visit a location twice in a row before a final target location is reached, i.e., it forgives $l1$ to be visited twice before $l2$. This constraint combined with the sequenced visit pattern allows the realization of a strict sequenced visit.";
		}
	},
	FAIR_VISIT {
		@Override
		public String toString() {
			return "Fair Visit";
		}

		@Override
		public LTLFormula getMission(String[] locations) {
			LTLFormula f = CoreMovementPatterns.VISIT.getMission(locations);

			List<String> loc = Arrays.asList(locations);
			LTLFormula f1 = LTLFormula.TRUE;

			for (int index = 0; index < loc.size(); index++) {
				LTLFormula f2 = LTLFormula.TRUE;
				for (int index2 = 0; index2 < loc.size(); index2++) {
					if (index != index2) {
						f2 = LTLFormula.and(f2,
								new LTLIUntil(new LTLINegation(new LTLIPropositionalAtom(loc.get(index))),
										new LTLIPropositionalAtom(loc.get(index2))));
					}
				}

				f1 = LTLFormula.and(f1,

						new LTLIGlobally(new LTLIImplies(new LTLIPropositionalAtom(loc.get(index)), f2

						)));
			}
			return new LTLConjunction(f, f1);
		}

		@Override
		public String getDescription() {
			return "Given a set of areas A the robot(s) should visit all the areas in the set A following a strict order.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\" where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space.";
		}

		@Override
		public String getExamples() {
			return "Consider the areas l1, l2, l3, and l4 and a set of areas {l1,l2,l3} to be all covered. If the a robot visits the areas l1, l4, l3, l1, l4, and l2, it does not perform a fair visit since it visits l1 twice while l2 and l3 are visited once. Vice versa, if the a robot visits the areas l1, l4, l3, l1, l4, l2, l2, and l4, it performs a fair visit since it visits l1, l2, and l3 twice.";
		}

		@Override
		public String getRelationships() {
			return "The Fair visit pattern specializes the Visit pattern by further constraining how locations are visited to ensure a fair visit.";
		}

		@Override
		public String getOccurrences() {
			return "Smith et al.proposed an LTL mission specification for the following mission requirement ``an equal number of visits to each data-gather location\" is required. The LTL mission specification is obtained by forcing an order on how the data-gathered locations are visited. However, fair visiting may be required even without the specification of an order in which the locations must be visited.";
		}
	},
	PATROLLING {
		@Override
		public String toString() {
			return "Patrolling";
		}

		@Override
		public LTLFormula getMission(String[] locations) {
			return new LTLIGlobally(CoreMovementPatterns.VISIT.getMission(locations));
		}

		@Override
		public String getDescription() {
			return "Intent	this pattern requires a robot to keep visiting a set of areas or points, but not in a particular order.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\" where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space. It can be considered in two variations: one version specifies that the robot is forced to cover all the areas or the robot should keep entering at least one of the areas in the specified set. The second version can be obtained from the template by replacing the AND operator with an OR";
		}

		@Override
		public String getExamples() {
			return "This pattern also appears in literature as surveillance. It is used to encode infinite executions of the robot, such as surveillance, persistent monitoring, and pickup-delivery missions.  Consider the areas l1, l2, l3, and l4 and a set of areas {l1,l2,l3} to be all surveilled. If a robot keeps entering the areas following the order l1, l4, l3, l1, l4, and l2, the mission is achieved. Vice versa if the robot keep visiting the areas as follows l1, l4, l3, l1, and l4 the mission is not achieved since l2 is not surveilled.";
		}

		@Override
		public String getRelationships() {
			return "The Patrolling pattern generalizes the Visit pattern by requiring to keep visiting a set of areas.";
		}

		@Override
		public String getOccurrences() {
			return "Wolff et al. provides an LTL specification to encode a mission requirement requiring that two locations must be visited repeatedly. It also propose an LTL property specifying that an agent should keep entering a location. These specifications can be considered as instances of the survillance pattern. Smith et al. proposed a mission requirement that forces robots to repeatedly visit a set of locations. This is an example of usage of the patrolling pattern.";
		}
	},
	SEQUENCED_PATROLLING {
		@Override
		public String toString() {
			return "Sequenced Patrolling";
		}

		@Override
		public LTLFormula getMission(String[] locations) {
			return new LTLIGlobally(CoreMovementPatterns.SEQUENCED_VISIT.getMission(locations));
		}

		@Override
		public String getDescription() {
			return "This pattern requires a robot to keep visiting a set of areas or points, in some specified order, similarly to sequenced visit.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\" where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space. It can be considered in two variations: one version specifies that the robot is forced to cover all the areas or the robot should keep entering at least one of the areas in the specified set. The second version can be obtained from the template by replacinge the AND operator with an OR";
		}

		@Override
		public String getExamples() {
			return "Consider the areas l1, l2, l3, and l4 and a set of areas {l1,l2,l3} to be all sequencing surveilled following the order l1, l2, and l3. If a robot keeps visiting the areas l1, l4, l3, l1, l4, and l2 in this order, the mission is satisfied, since globally any l1 will be finally followed by l2 and l2 by l3.";
		}

		@Override
		public String getRelationships() {
			return "The Sequenced Patrolling pattern specializes the Patrolling by requiring locations to be visited in sequence.";
		}

		@Override
		public String getOccurrences() {
			return "An example of mission specification that solves the sequence patrolling problem has been provided by Chen et al. that proposed a formula specifying that two tasks are performed in sequences, and one of them must occur at least once before the other.";
		}
	},
	ORDERED_PATROLLING {
		@Override
		public String toString() {
			return "Ordered Patrolling";
		}

		@Override
		public LTLFormula getMission(String[] locations) {
			LTLFormula f = CoreMovementPatterns.SEQUENCED_PATROLLING.getMission(locations);

			List<String> loc = Arrays.asList(locations);
			LTLFormula f1 = LTLFormula.TRUE;

			
			for (int index = loc.size() - 1; index > 0; index--) {
				f1 = LTLFormula.and(f1, new LTLIUntil(new LTLINegation(new LTLIPropositionalAtom(loc.get(index))),
						new LTLIPropositionalAtom(loc.get(index-1))));
			}
			return new LTLConjunction(f, f1);
		}

		@Override
		public String getDescription() {
			return "this pattern requires a robot to keep visiting a set of areas or points, in some specified order, similarly to sequenced patrolling. However, given an order, e.g., l1 and l2 between two locations, it is not admitted to the robot to visit l2 before l1.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l, and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\" where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space.";
		}

		@Override
		public String getExamples() {
			return "Consider the areas l1, l2, l3, and l4 and a set of areas {l1,l2,l3} to be all patrolled following the order l1, l2, and l3. If the a robot keep visiting the areas l1, l4, l3, l1, l4, and l2 in this order, the mission is not satisfied, since l3 precedes l2. Differently, if the robot keeps visiting the areas l1, l1, l2, l4, l4, and l3 in this order, the mission is satisfied.";
		}

		@Override
		public String getRelationships() {
			return "The Ordered Patrolling pattern specializes Sequenced Patrolling by requiring locations to be visited in order.";
		}

		@Override
		public String getOccurrences() {
			return "An example of ordered patrolling can be obtained by combining the sequence patrolling formulation has been provided by Chen et. al.Chen et al., with the absence pattern proposed by Dwyer et al..";
		}
	},
	STRICT_ORDERED_PATROLLING {
		@Override
		public String toString() {
			return "Strict Ordered Patrolling";
		}

		@Override
		public LTLFormula getMission(String[] locations) {
			LTLFormula f = CoreMovementPatterns.ORDERED_PATROLLING.getMission(locations);

			List<String> loc = Arrays.asList(locations);
			LTLFormula f1 = LTLFormula.TRUE;

			for (int index = 0; index < loc.size()-1; index++) {
				f1 = LTLFormula.and(f1, 
						new LTLIGlobally(
								new LTLIImplies(
										new LTLIPropositionalAtom(loc.get(index)),
										new LTLNext(
												new LTLIUntil(
														new LTLINegation(new LTLIPropositionalAtom(loc.get(index))),
														new LTLIPropositionalAtom(loc.get(index+1))
														)
												)
										)
								)
					);
			}
			return new LTLConjunction(f, f1);
		}

		@Override
		public String getDescription() {
			return "A robot must patrol an area following a strict order. This area can be a set of locations of a building, or a set of points of interest on a map that must be surveyed.";
		}

		@Override
		public String getVariations() {
			return "Variation	If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l, and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\", where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space.";
		}

		@Override
		public String getExamples() {
			return "A common usage example of the Strict Sequence Patrolling pattern is a scenario where a robot is performing surveillance in a building during night hours. Strict Sequence Patrolling and Avoidance often go together. Avoidance patterns are used to force robots to avoid obstacles as they guard an area. Triggers can also be used in combination with the Strict Sequence Patrolling pattern to specify conditions upon which Patrolling should start or stop.";
		}

		@Override
		public String getRelationships() {
			return "Relationships: The Strict Sequence Patrolling pattern is a specialisation of the Visiting pattern, in which the robot should keep visiting a set of points or locations in a given strict order.";
		}

		@Override
		public String getOccurrences() {
			return "Smith et al. proposed a mission specification that forces a robots to not visit a location twice in a row before a final target location is reached. This can be considered as an example of strict patrolling.";
		}
	},
	FAIR_PATROLLING {
		@Override
		public String toString() {
			return "Fair Patrolling";
		}

		@Override
		public LTLFormula getMission(String[] locations) {
			LTLFormula f = new LTLIGlobally(CoreMovementPatterns.ORDERED_VISIT.getMission(locations));

			List<String> loc = Arrays.asList(locations);
			LTLFormula f1 = LTLFormula.TRUE;
			LTLFormula f2 = LTLFormula.TRUE;

			for (int index = 0; index < loc.size() - 1; index++) {
				f1 = LTLFormula.and(f1,
						new LTLIGlobally(new LTLIImplies(LTLFormula.and(new LTLIPropositionalAtom(loc.get(index)),
								new LTLINegation(new LTLIPropositionalAtom(loc.get(index))

								)), new LTLIUntil(new LTLINegation(new LTLIPropositionalAtom(loc.get(index))), new LTLIPropositionalAtom(loc.get(index + 1))))));
				f2 = LTLFormula.and(f2, new LTLIUntil(new LTLINegation(new LTLIPropositionalAtom(loc.get(index))),
						new LTLIPropositionalAtom(loc.get(index + 1))));
			}
			return new LTLConjunction(f, new LTLConjunction(f2, f1));

		}

		@Override
		public String getDescription() {
			return "This pattern requires a robot to keep visiting a set of areas or points in a fair way, i.e., the robots patrols the area by keeping the number of times every area is patrolled equal.";
		}

		@Override
		public String getVariations() {
			return "If a relational notion of space is used, propositions have the form \"r in l\" where in indicates that the robot r is inside location l and l identifies the desired location. If an absolute notion of space is used, propositions have the form \"r at (x,y,z)\", where at indicates that the robot r is in a specific point and (x,y,z) indicates a precise position in space.";
		}

		@Override
		public String getExamples() {
			return "Consider the areas l1, l2, l3, and l4 and a set of areas {l1,l2,l3} to be all patrolled. If the robot patrols the areas l1, l4, l3, l1, l4, and l2, it does not perform a fair visit since it patrols l1 more than it patrols l2 and l3. Vice versa, if the a robot patrols the areas l1, l4, l3, l1, l4, l2, l2, and l4, it performs a fair visit since it patrols l1, l2, and l3 fairly.";
		}

		@Override
		public String getRelationships() {
			return "	Relationships: The Fair Patrolling pattern is a specialization of the Patrolling pattern, in which the robot should keep visiting a set of locations in a fair way.";
		}

		@Override
		public String getOccurrences() {
			return "	Smith et al. proposed an instance of mission specification that requires an equal number of visits to each data-gathered location.";
		}
	};
	public abstract LTLFormula getMission(String[] locations);

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
