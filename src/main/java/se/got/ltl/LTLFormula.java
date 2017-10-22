package se.got.ltl;

import java.util.Set;

import com.google.common.base.Preconditions;

import se.got.ltl.atoms.LTLIPropositionalAtom;
import se.got.ltl.atoms.MITLITrue;
import se.got.ltl.visitors.MITLIVisitor;

public abstract class LTLFormula extends Formula {

	private int maxIntComparedto = 0;

	public static final LTLFormula TRUE = new MITLITrue();

	public LTLFormula() {
		super();

	}

	public abstract Set<LTLFormula> getChildren();

	public abstract <T> T accept(MITLIVisitor<T> visitor);

	public int maxIntComparedto() {
		return maxIntComparedto;
	}

	public int maxIntComparedto(int c) {
		if (maxIntComparedto < c)
			maxIntComparedto = c;

		return maxIntComparedto;
	}

	// Producers method to build CLTL formulae of the argument formula

	public static LTLFormula atom(String s) {
		return new LTLIPropositionalAtom(s);
	}

	public static LTLFormula not(LTLFormula f) {
		if (f instanceof LTLINegation) {
			return ((LTLINegation) f).getChild();
		}

		return new LTLINegation(f);
	}

	public static LTLFormula and(LTLFormula... formulae) {

		LTLFormula f = formulae[0];
		for (int i = 1; i < formulae.length; i++) {
			if (f.equals(LTLFormula.TRUE)) {
				f = formulae[i];
			} else {
				f = new LTLConjunction(f, formulae[i]);
			}
		}
		return f;
	}

	public static LTLFormula U(LTLFormula f1, LTLFormula f2) {
		return new LTLIUntil(f1, f2);
	}

	public static LTLFormula S(LTLFormula f1, LTLFormula f2) {
		return new LTLISince(f1, f2);
	}

	// Producers method to build derived boolean CLTL formulae
	public static LTLFormula or(LTLFormula... formulae) {
		LTLFormula f = formulae[0];
		for (int i = 1; i < formulae.length; i++) {
			f = new LTLIDisjunction(f, formulae[i]);
		}
		return f;
	}

	public static LTLFormula implies(LTLFormula f1, LTLFormula f2) {
		Preconditions.checkNotNull(f1, "The first subformula cannot be null");
		Preconditions.checkNotNull(f2, "The second subformula cannot be null");

		return new LTLIImplies(f1, f2);
	}

	public static LTLFormula iff(LTLFormula f1, LTLFormula f2) {
		return new LTLIIff(f1, f2);
	}

	// Producers method to build derived temporal CLTL formulae
	public static LTLFormula R(LTLFormula f1, LTLFormula f2) {
		return not(U(not(f1), not(f2)));
	}

}
