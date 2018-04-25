package se.got.ltl.visitors;

import se.got.ltl.LTLConjunction;
import se.got.ltl.LTLIDisjunction;
import se.got.ltl.LTLEventually;
import se.got.ltl.LTLIGlobally;
import se.got.ltl.LTLIIff;
import se.got.ltl.LTLIImplies;
import se.got.ltl.LTLINegation;
import se.got.ltl.LTLISince;
import se.got.ltl.LTLIUntil;
import se.got.ltl.LTLNext;
import se.got.ltl.atoms.LTLIPropositionalAtom;
import se.got.ltl.atoms.MITLIRelationalAtom;
import se.got.ltl.atoms.MITLITrue;

public class LTLFormulaToStringVisitor implements MITLIVisitor<String> {

	@Override
	public String visit(LTLIDisjunction formula) {
		return "( "+formula.getLeftChild().accept(this)+") || ("+formula.getRightChild().accept(this)+" )";
	}

	@Override
	public String visit(MITLITrue formula) {
		return "TRUE";
}

	@Override
	public String visit(LTLConjunction formula) {
		return "( "+formula.getLeftChild().accept(this)+") && ("+formula.getRightChild().accept(this)+" )";
	}

	@Override
	public String visit(LTLINegation formula) {
		return " ! (" + formula.getChild().accept(this) +")";
	}

	@Override
	public String visit(LTLIUntil formula) {
		return "( "+formula.getLeftChild().accept(this)+") U ("+formula.getRightChild().accept(this)+" )";
	}

	@Override
	public String visit(LTLIImplies formula) {
		return "( "+formula.getLeftChild().accept(this)+") -> ("+formula.getRightChild().accept(this)+" )";
	}

	@Override
	public String visit(LTLIIff formula) {
		return "( "+formula.getLeftChild().accept(this)+") <-> ("+formula.getRightChild().accept(this)+" )";
	}

	@Override
	public String visit(LTLISince formula) {
		return "( "+formula.getLeftChild().accept(this)+") S ("+formula.getRightChild().accept(this)+" )";
	}

	@Override
	public String visit(LTLIPropositionalAtom formula) {
		return formula.getAtomName();
	}

	@Override
	public String visit(MITLIRelationalAtom formula) {
		return formula.getIdentifier();
	}

	@Override
	public String visit(LTLEventually mitliEventually) {
		return " <> (" + mitliEventually.getChild().accept(this) +")";
	}

	@Override
	public String visit(LTLIGlobally mitliGlobally) {
		return " [] (" + mitliGlobally.getChild().accept(this) +")";
	}

	@Override
	public String visit(LTLNext ltlNext) {
		return " X (" + ltlNext.getChild().accept(this) +")";
	}

}
