package se.got.ltl;

import java.util.HashSet;
import java.util.Set;

import com.google.common.base.Preconditions;

import se.got.ltl.visitors.MITLIVisitor;

public class LTLNext extends LTLFormula implements UnaryFormula<LTLFormula> {

	// Formula f1 which is argument of the negation (\neg f1)
	private LTLFormula subformula;

	private final String operator = "NEXT";

	public LTLNext(LTLFormula subformula) {

		Preconditions.checkNotNull(subformula, "The subformula cannot be null");
		this.subformula = subformula;
	}

	@Override
	public <T> T accept(MITLIVisitor<T> visitor) {
		return visitor.visit(this);
	}

	@Override
	public LTLFormula getChild() {
		return this.subformula;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toString() {
		return "(" + this.operator + this.getChild() + ")";
	}
	
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((operator == null) ? 0 : operator.hashCode());
		result = prime * result + ((subformula == null) ? 0 : subformula.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		LTLNext other = (LTLNext) obj;
		if (operator == null) {
			if (other.operator != null)
				return false;
		} else if (!operator.equals(other.operator))
			return false;
		if (subformula == null) {
			if (other.subformula != null)
				return false;
		} else if (!subformula.equals(other.subformula))
			return false;
		return true;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Set<LTLFormula> getChildren() {
		Set<LTLFormula> formulae=new HashSet<>();
		formulae.add(subformula);
		return formulae;
	}
}
