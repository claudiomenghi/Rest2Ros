package se.got.ltl;

import java.util.HashSet;
import java.util.Set;

import com.google.common.base.Preconditions;

import se.got.ltl.visitors.MITLIVisitor;

public class LTLIUntil extends LTLFormula implements BinaryFormula<LTLFormula> {

	private final LTLFormula subformula1;
	private final LTLFormula subformula2;
	
	private final String operator = "U";

	public LTLIUntil(LTLFormula subformula1, LTLFormula subformula2) {
		super();
		Preconditions.checkNotNull(subformula1, "The first subformula cannot be null");
		Preconditions.checkNotNull(subformula2, "The second subformula cannot be null");
		this.subformula1 = subformula1;
		this.subformula2 = subformula2;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public LTLFormula getLeftChild() {
		return this.subformula1;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public LTLFormula getRightChild() {
		return this.subformula2;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public <T> T accept(MITLIVisitor<T> visitor) {
		return visitor.visit(this);
	}
	
	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toString() {
		return "(" + this.subformula1 + " " + operator + " " + this.subformula2 + ")";
	}
	
	/**
	 * {@inheritDoc}
	 */
	@Override
	public Set<LTLFormula> getChildren() {
		Set<LTLFormula> formulae=new HashSet<>();
		formulae.add(subformula1);
		formulae.add(subformula2);
		return formulae;
	}
}
