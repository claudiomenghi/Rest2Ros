package se.got;

import java.util.function.BinaryOperator;

import se.got.ltl.LTLConjunction;
import se.got.ltl.LTLFormula;

public enum Composition {

	AND {
		@Override
		public LTLFormula getMission(LTLFormula f1, LTLFormula f2) throws Exception {
			return LTLFormula.and(f1, f2);
		}

		@Override
		public String getDescription() {
			return "AND combines two missions";
		}

	},
	OR {
		@Override
		public LTLFormula getMission(LTLFormula f1, LTLFormula f2) throws Exception {
			return LTLFormula.or(f1, f2);
		}

		@Override
		public String getDescription() {
			return "Creates the OR combination of two missions";
		}

	};

	public abstract LTLFormula getMission(LTLFormula f1, LTLFormula f2) throws Exception;

	public abstract String getDescription();

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
